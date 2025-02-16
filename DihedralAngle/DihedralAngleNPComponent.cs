using System;
using System.Collections.Generic;
using System.Linq;
using System.Drawing;
using Grasshopper.Kernel;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;

namespace DihedralAngle
{
    public class DihedralAngleNPComponent : GH_Component
    {
        // fields
        private List<Point3d> pointsForDisplay;
        private List<int> edgesIndexesForDisplay;
        private bool switchVisualise;

        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public DihedralAngleNPComponent()
          : base("DihedralAngleNP", "DANP",
              "\"Calculate Internal Angle of closed Brep",
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
            pManager.AddBrepParameter("Brep", "B", "A planar Brep to be evaluated", GH_ParamAccess.item);
            pManager.AddNumberParameter("Parameters", "P", "A list of parameters for the point in the edge - if none is provided, 0.5 will be input", GH_ParamAccess.list);
            pManager[1].Optional = true;
            pManager.AddBooleanParameter("Visualise", "V", "A switch for edge index visualisation", GH_ParamAccess.item);
            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Angles Rad", "AR", "List of Angles in Radians - associated with edge index", GH_ParamAccess.list);
            pManager.AddGenericParameter("Angles Deg", "AD", "List of Angles in Degrees - associated with edge index", GH_ParamAccess.list);
            pManager.AddPointParameter("Points on Edges", "P", "List of Points on Edges - associated with edge index", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            DA.GetData(2, ref switchVisualise);

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

            List<double> parameters = new List<double>();
            DA.GetData(1, ref parameters);
            CheckList(body.Edges.Count, ref parameters);

            foreach (BrepEdge edge in body.Edges)
            {
                Point3d testPointOnEdge = edge.PointAt(parameters[edge.EdgeIndex]);

                int[] neighbourFacesIndexes = edge.AdjacentFaces();

                List<BrepFace> neighbourFaces = body.Faces.Where(f => neighbourFacesIndexes.Contains(f.FaceIndex)).ToList();

                if (neighbourFaces.Count > 2)
                {
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Part of the Brep is non Manifold");
                    return;
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
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resource1.DihedralAngleNP;
                //return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("F9918EA1-40F1-4DF0-ACAD-23A42DC8D4F4"); }
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

        public void CheckList(int count, ref List<double> list)
        {
            if (list.Count == 0)
            {
                list.AddRange(Enumerable.Repeat(0.5, count));
            }
            else if (list.Count < count)
            {
                list.AddRange(Enumerable.Repeat(0.5, list.Count - count));
            }
            else if (list.Count > count)
            {
                list.RemoveRange(count - 1, list.Count - 1);
            }
        }

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