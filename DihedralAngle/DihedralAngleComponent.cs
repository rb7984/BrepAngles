using Grasshopper.GUI.SettingsControls;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Geometry.Delaunay;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.UI;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace DihedralAngle
{
    public class DihedralAngleComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public DihedralAngleComponent()
          : base("DihedralAngle", "DA",
              "Calculate Internal Angle of closed Planar Brep",
              "Surface", "Util")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "A planar Brep to be evaluated", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Indexes", "I", "List of edge Indexes", GH_ParamAccess.list);
            pManager.AddGenericParameter("Angles Rad", "AR", "List of Angles in Radians", GH_ParamAccess.list);
            pManager.AddGenericParameter("Angles Deg", "AD", "List of Angles in Degrees", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //TODO add control for open Breps
            List<double> edgeIndexes = new List<double>();
            List<double> radians = new List<double>();
            List<double> degrees = new List<double>();
            List<double> dimensions = new List<double>();

            //TODO Add a display for Angular Dimensions
            //List<AngularDimension> dimensions = new List<AngularDimension>();
            //List<AngularDimensionObject> dimensionsObjects = new List<AngularDimensionObject>();

            Brep body = new Brep();
            DA.GetData(0, ref body);

            foreach (BrepFace face in body.Faces)
            {
                if (!face.IsPlanar())
                {
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The Brep contains Faces that are not planar");
                    return;
                }
            }

            foreach (BrepEdge edge in body.Edges)
            {
                Curve edgeCurve = edge.DuplicateCurve();
                edgeCurve.Domain = new Interval(0, 1);

                int[] neighbourFacesIndexes = edge.AdjacentFaces();

                List<BrepFace> neighbourFaces = body.Faces.Where(f => neighbourFacesIndexes.Contains(f.FaceIndex)).ToList();

                if (neighbourFaces.Count > 2)
                {
                    //TODO Add runtime message for non manifold brep
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Part of the Brep is non Manifold");
                    break;
                }
                else
                {
                    BrepFace face = neighbourFaces[0];
                    Vector3d faceNormal = face.NormalAt(0.5, 0.5);

                    Curve edgeAsCurve = edge.ToNurbsCurve();
                    double parameter;
                    edgeAsCurve.ClosestPoint(face.GetBoundingBox(true).Center, out parameter);

                    Vector3d scalingVector = edgeAsCurve.PointAt(parameter) - face.GetBoundingBox(true).Center;

                    scalingVector.Unitize();

                    AngularDimension d;
                    double dihedralAngle = PreCalculate(edge, faceNormal, face, neighbourFaces[1], out d);

                    //TODO put a dosèòay for edges
                    //Rhino.RhinoDoc.ActiveDoc.Objects.Add(new TextDot(edge.EdgeIndex.ToString(), edge.GetBoundingBox(false).Center));

                    edgeIndexes.Add(edge.EdgeIndex);
                    radians.Add(dihedralAngle);
                    degrees.Add(RhinoMath.ToDegrees(dihedralAngle));

                    //strings.Add(face.FaceIndex.ToString() + "-" + edge.EdgeIndex.ToString() + ": " + dihedralAngle.ToString());
                    //dimensions.Add(d);
                }
            }

            DA.SetDataList(0, edgeIndexes);
            DA.SetDataList(1, radians);
            DA.SetDataList(2, degrees);
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resource1.DihedralAngle;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("f5339064-5447-4e67-9be4-7d663dc3a224"); }
        }

        #region Auxiliary Methods
        public double PreCalculate(BrepEdge edge, Vector3d faceNormal, BrepFace face, BrepFace adjacentFace, out AngularDimension ad)
        {
            Vector3d adjacentFaceNormal = adjacentFace.NormalAt(0.5, 0.5);

            var loop = face.OuterLoop;
            Curve loopasacurve = loop.To3dCurve();
            Polyline loopasapolyline = new Polyline();

            if (loopasacurve.IsPolyline())
            {
                //TODO maximum length può causare dei danni?
                loopasapolyline = loopasacurve.ToPolyline(0.1, 0.1, 0.1, 3000).ToPolyline();
            }

            loopasapolyline.MergeColinearSegments(0.1, true);
            Line[] ll = loopasapolyline.GetSegments();
            Line line = new Line();

            double distance = 100;
            int counter = 0;
            while (distance > 1)
            {
                double t;
                if (edge.ClosestPoint(ll[counter].PointAt(0.5), out t))
                {
                    distance = edge.PointAt(t).DistanceTo(ll[counter].PointAt(0.5));
                    line = ll[counter];
                    counter++;
                }
            }

            Vector3d testEdgeVector = line.Direction;
            testEdgeVector.Unitize();

            Vector3d rotatedFaceNormal = new Vector3d(faceNormal);
            rotatedFaceNormal.Rotate(Math.PI / 2, testEdgeVector);

            double rotatedDotProduct = Vector3d.Multiply(rotatedFaceNormal, adjacentFaceNormal);

            Vector3d vectorA = Vector3d.CrossProduct(faceNormal, testEdgeVector);
            vectorA.Unitize();
            Vector3d vectorB = Vector3d.CrossProduct(adjacentFaceNormal, -testEdgeVector);
            vectorB.Unitize();

            Arc arc = new Arc(pointA: line.PointAt(0.5) + (vectorA * line.Length * 0.5), tangentA: vectorA, pointB: line.PointAt(0.5) + (vectorB * line.Length * 0.5));

            ad = new AngularDimension(arc, line.Length * 0.5);

            return CalculateDihedralAngle(faceNormal, adjacentFaceNormal, rotatedDotProduct);
        }

        public double CalculateDihedralAngle(Vector3d normal1, Vector3d normal2, double dotProductRotated)
        {
            normal1.Unitize();
            normal2.Unitize();

            double angle = Vector3d.VectorAngle(normal1, normal2);

            if (dotProductRotated > 0)
            {
                angle = Math.PI - angle;
            }
            else
            {
                angle = Math.PI + angle;
            }

            return angle;
        }

        #endregion
    }
}
