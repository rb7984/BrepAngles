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
using System.Drawing;
using System.Linq;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace BrepAngles
{
    public class DihedralAngleComponent : GH_Component
    {
        // fields
        private List<Point3d> pointsForDisplay;
        private List<int> edgesIndexesForDisplay;
        private bool switchVisualise;

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
            pointsForDisplay = new List<Point3d>();
            edgesIndexesForDisplay = new List<int>();
            switchVisualise = false;
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "A planar Brep to be evaluated.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Visualise", "V", "A switch for edge index visualisation.", GH_ParamAccess.item);
            pManager[1].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Angles Rad", "AR", "List of Angles in Radians - associated with edge index.", GH_ParamAccess.list);
            pManager.AddGenericParameter("Angles Deg", "AD", "List of Angles in Degrees - associated with edge index.", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            DA.GetData(1, ref switchVisualise);

            // Reset the display Lists
            pointsForDisplay.Clear();
            edgesIndexesForDisplay.Clear();

            List<double> radians = new List<double>();
            List<double> degrees = new List<double>();

            //TODO Add a display for Angular Dimensions
            //List<AngularDimension> dimensions = new List<AngularDimension>();
            //List<AngularDimensionObject> dimensionsObjects = new List<AngularDimensionObject>();

            Brep body = new Brep();
            DA.GetData(0, ref body);

            foreach (BrepFace face in body.Faces)
            {
                if (!face.IsPlanar())
                {
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The Brep contains Faces that are not planar.");
                    return;
                }
            }

            foreach (BrepEdge edge in body.Edges)
            {
                int[] neighbourFacesIndexes = edge.AdjacentFaces();

                List<BrepFace> neighbourFaces = body.Faces.Where(f => neighbourFacesIndexes.Contains(f.FaceIndex)).ToList();

                if (neighbourFaces.Count > 2)
                {
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Part of the Brep is non Manifold.");
                    return;
                }
                else
                {
                    BrepFace face = neighbourFaces[0];
                    Vector3d faceNormal = face.NormalAt(0.5, 0.5);

                    AngularDimension d;
                    //CLEAN facenormal can go inside the method
                    double dihedralAngle = Calculate(edge, faceNormal, face, neighbourFaces[1], out d);

                    pointsForDisplay.Add(edge.GetBoundingBox(false).Center);
                    edgesIndexesForDisplay.Add(edge.EdgeIndex);

                    radians.Add(dihedralAngle);
                    degrees.Add(RhinoMath.ToDegrees(dihedralAngle));
                }
            }

            DA.SetDataList(0, radians);
            DA.SetDataList(1, degrees);
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

        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            if (switchVisualise)
                for (int i = 0; i < pointsForDisplay.Count; i++)
                {
                    Plane plane;
                    args.Viewport.GetCameraFrame(out plane);
                    plane.Origin = pointsForDisplay[i];

                    double pixelsPerUnit;
                    args.Viewport.GetWorldToScreenScale(pointsForDisplay[i], out pixelsPerUnit);
                    args.Display.Draw3dText(edgesIndexesForDisplay[i].ToString(), Color.Black, plane, 25 / pixelsPerUnit, "Lucida Console", false, false, TextHorizontalAlignment.Center, TextVerticalAlignment.Middle);
                }

            //base.DrawViewportMeshes(args);
        }

        #region Auxiliary Methods
        public double Calculate(BrepEdge edge, Vector3d faceNormal, BrepFace face, BrepFace adjacentFace, out AngularDimension ad)
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
