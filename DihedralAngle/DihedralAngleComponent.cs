using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.UI;
using System;
using System.Collections.Generic;
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
          : base("DihedralAngle", "Nickname",
              "Description",
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
            pManager.AddGenericParameter("Output", "O", "List of Angles", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<string> strings = new List<string>();

            Brep buildingBody = new Brep();
            DA.GetData(0, ref buildingBody);

            foreach (BrepFace face in buildingBody.Faces)
            {
                int[] edgesIndex = face.AdjacentEdges();

                List<BrepEdge> surroundingEdges = buildingBody.Edges.Where(e => edgesIndex.Contains(e.EdgeIndex)).ToList();

                Vector3d faceNormal = face.NormalAt(0.5, 0.5);

                Rhino.RhinoDoc.ActiveDoc.Objects.Add(new TextDot(face.FaceIndex.ToString(), face.GetBoundingBox(false).Center));

                foreach (BrepEdge edge in surroundingEdges)
                {
                    Curve edgeCurve = edge.DuplicateCurve();
                    edgeCurve.Domain = new Interval(0, 1);

                    int[] neighbourFacesByFace = face.AdjacentFaces();
                    int[] neighbourFacesByEdge = edge.AdjacentFaces();

                    int[] neighbourFaces = neighbourFacesByEdge.Intersect(neighbourFacesByFace).ToArray();

                    List<BrepFace> neighbourFace = buildingBody.Faces.Where(f => neighbourFaces.Contains(f.FaceIndex)).ToList();

                    Curve edgeAsCurve = edge.ToNurbsCurve();
                    double parameter;
                    edgeAsCurve.ClosestPoint(face.GetBoundingBox(true).Center, out parameter);

                    Vector3d scalingVector = edgeAsCurve.PointAt(parameter) - face.GetBoundingBox(true).Center;

                    scalingVector.Unitize();

                    double dihedralAngle = PreCalculate(edge, faceNormal, face, neighbourFace[0]);

                    Rhino.RhinoDoc.ActiveDoc.Objects.Add(new TextDot(edge.EdgeIndex.ToString(), edge.GetBoundingBox(false).Center));

                    strings.Add(face.FaceIndex.ToString() + "-" + edge.EdgeIndex.ToString() + ": " + dihedralAngle.ToString());
                }
            }

            DA.SetDataList(0, strings);
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return null;
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
        public double PreCalculate(BrepEdge edge, Vector3d faceNormal, BrepFace face, BrepFace adjacentFace)
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
