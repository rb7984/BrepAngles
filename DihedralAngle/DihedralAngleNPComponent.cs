using System;
using System.Collections.Generic;
using System.Linq;
using System.Drawing;
using Grasshopper.Kernel;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;
using System.Runtime.InteropServices;
using Grasshopper.Kernel.Geometry.Delaunay;

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

            pManager.AddVectorParameter("Vectors A", "VA", "va", GH_ParamAccess.list);
            pManager.AddVectorParameter("Vectors B", "VB", "vb", GH_ParamAccess.list);
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
            List<Point3d> pointsOnEdges = new List<Point3d>();

            //TODO Add a display for Angular Dimensions
            //List<AngularDimension> dimensions = new List<AngularDimension>();
            //List<AngularDimensionObject> dimensionsObjects = new List<AngularDimensionObject>();

            List<Vector3d> vectorsA = new List<Vector3d>();
            List<Vector3d> vectorsB = new List<Vector3d>();

            Brep body = new Brep();
            DA.GetData(0, ref body);

            List<double> parameters = new List<double>();
            DA.GetDataList(1, parameters);
            CheckList(body.Edges.Count, ref parameters);

            foreach (BrepEdge edge in body.Edges)
            {
                double parameter = edge.Domain.T0 + (edge.Domain.Length * parameters[edge.EdgeIndex]);

                Point3d testPointOnEdge = edge.PointAt(parameter);
                pointsOnEdges.Add(testPointOnEdge);

                int[] neighbourFacesIndexes = edge.AdjacentFaces();

                List<BrepFace> neighbourFaces = body.Faces.Where(f => neighbourFacesIndexes.Contains(f.FaceIndex)).ToList();

                if (neighbourFaces.Count > 2)
                {
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Part of the Brep is non Manifold");
                    return;
                }
                else
                {
                    BrepFace face0 = neighbourFaces[0];

                    double u0, v0;
                    face0.ClosestPoint(testPointOnEdge, out u0, out v0);

                    Vector3d faceNormal0 = face0.NormalAt(u0, v0);

                    BrepFace face1 = neighbourFaces[1];

                    double u1, v1;
                    face1.ClosestPoint(testPointOnEdge, out u1, out v1);

                    Vector3d faceNormal1 = face1.NormalAt(u1, v1);

                    //Calculate
                    AngularDimension d;
                    Vector3d va;
                    Vector3d vb;
                    double dihedralAngle = Calculate(edge, testPointOnEdge, faceNormal0, faceNormal1, face0, out d, out va, out vb);

                    pointsForDisplay.Add(edge.GetBoundingBox(false).Center);
                    edgesIndexesForDisplay.Add(edge.EdgeIndex);

                    radians.Add(dihedralAngle);
                    degrees.Add(RhinoMath.ToDegrees(dihedralAngle));

                    vectorsA.Add(va);
                    vectorsB.Add(vb);
                }
            }

            DA.SetDataList(0, radians);
            DA.SetDataList(1, degrees);
            DA.SetDataList(2, pointsOnEdges);
            DA.SetDataList(3, vectorsA);
            DA.SetDataList(4, vectorsB);
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
            // TODO parameters should all be values between 0 and 1
            if (list.Count == 0)
            {
                list.AddRange(Enumerable.Repeat(0.5, count));
            }
            else if (list.Count < count)
            {
                list.AddRange(Enumerable.Repeat(list[list.Count - 1], count - list.Count));
            }
            else if (list.Count > count)
            {
                list.RemoveRange(count - 1, list.Count - 1);
            }
        }

        public double Calculate(BrepEdge edge, Point3d testPoint, Vector3d faceNormal1, Vector3d faceNormal2, BrepFace face, out AngularDimension ad, out Vector3d va, out Vector3d vb)
        {
            var loop = face.OuterLoop;
            Curve loopasacurve = loop.To3dCurve();

            double t = 0;
            loopasacurve.ClosestPoint(testPoint, out t);

            Vector3d testEdgeVector = loopasacurve.TangentAt(t);
            testEdgeVector.Unitize();

            Vector3d rotatedFaceNormal = new Vector3d(faceNormal1);
            rotatedFaceNormal.Rotate(Math.PI / 2, testEdgeVector);

            double rotatedDotProduct = Vector3d.Multiply(rotatedFaceNormal, faceNormal2);

            Vector3d vectorA = Vector3d.CrossProduct(faceNormal1, testEdgeVector);
            vectorA.Unitize();
            Vector3d vectorB = Vector3d.CrossProduct(faceNormal2, -testEdgeVector);
            vectorB.Unitize();

            Arc arc = new Arc(pointA: testPoint + (vectorA * loopasacurve.GetLength() * 0.2), tangentA: vectorA, pointB: testPoint + (vectorB * loopasacurve.GetLength() * 0.2));

            va = vectorA;
            vb = vectorB;
            ad = new AngularDimension(arc, loopasacurve.GetLength() * 0.5);

            return CalculateDihedralAngle(faceNormal1, faceNormal2, rotatedDotProduct);
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