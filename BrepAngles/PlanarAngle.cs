using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

using Grasshopper.Kernel;
using Grasshopper;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;

namespace BrepAngles
{
    public class PlanarAngle : GH_Component
    {
        // fields
        private List<Point3d> vertexPointsForDisplay;
        private List<int> faceIndexesForDisplay;

        private List<Point3d> facePointsForDisplay;
        private List<int> vertexIndexesForDisplay;

        private List<Arc> dimensionsArcsForDisplay;
        private List<double> dimensionsValuesForDisplay;

        private bool switchVisualise;
        private const int angulardimensionOffset = 50;

        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public PlanarAngle()
          : base("Planar Angle", "PA",
              "Calculate all Angles of every Brep's face",
              "Surface", "Util")
        {
            vertexPointsForDisplay = new List<Point3d>();
            vertexIndexesForDisplay = new List<int>();

            facePointsForDisplay = new List<Point3d>();
            faceIndexesForDisplay = new List<int>();

            dimensionsArcsForDisplay = new List<Arc>();
            dimensionsValuesForDisplay = new List<double>();

            switchVisualise = false;
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "A planar Brep to be evaluated.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Visualise", "V", "A switch for vertex index visualisation.", GH_ParamAccess.item);
            pManager[1].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Vertex Indexes", "I", "Tree of Indexes for the Angles.", GH_ParamAccess.tree);
            pManager.AddGenericParameter("Angles Rad", "AR", "Tree of Angles in Radians - associated with face/vertex index.", GH_ParamAccess.tree);
            pManager.AddGenericParameter("Angles Deg", "AD", "Tree of Angles in Degrees - associated with face/vertex index.", GH_ParamAccess.tree);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Reset the display Lists
            vertexPointsForDisplay.Clear();
            vertexIndexesForDisplay.Clear();

            facePointsForDisplay.Clear();
            faceIndexesForDisplay.Clear();

            dimensionsArcsForDisplay.Clear();
            dimensionsValuesForDisplay.Clear();

            // Output
            Grasshopper.DataTree<double> treevertexIndexes = new Grasshopper.DataTree<double>();
            Grasshopper.DataTree<double> treeAnglesDeg = new Grasshopper.DataTree<double>();
            Grasshopper.DataTree<double> treeAnglesRad = new Grasshopper.DataTree<double>();

            DA.GetData(1, ref switchVisualise);

            Brep body = new Brep();
            DA.GetData(0, ref body);

            if (body == null || body.Faces.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Invalid Brep input.");
                return;
            }

            foreach (BrepFace face in body.Faces)
                if (!face.IsPlanar())
                {
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The Brep contains Faces that are not planar.");
                    return;
                }

            foreach (BrepFace face in body.Faces)
            {
                facePointsForDisplay.Add(face.GetBoundingBox(false).Center);
                faceIndexesForDisplay.Add(face.FaceIndex);

                List<double> vertexIndexes = new List<double>();
                List<double> anglesRad = new List<double>();
                List<double> anglesDeg = new List<double>();

                Vector3d faceNormal = face.NormalAt(0.5, 0.5);

                var loop = face.OuterLoop;
                Curve loopasacurve = loop.To3dCurve();
                Polyline loopasapolyline = new Polyline();

                if (loopasacurve.IsPolyline())
                {
                    loopasapolyline = loopasacurve.ToPolyline(0.1, 0.1, 0.1, loopasacurve.GetLength()).ToPolyline();
                }

                loopasapolyline.MergeColinearSegments(0.1, true);
                Line[] ll = loopasapolyline.GetSegments();

                int[] faceAdjacentEdgesIndexes = face.AdjacentEdges();
                var faceAdjacentEdges = body.Edges.Where(e => faceAdjacentEdgesIndexes.Contains(e.EdgeIndex));

                Dictionary<BrepEdge, Vector3d> edgeDirectionDictionary = new Dictionary<BrepEdge, Vector3d>();

                foreach (BrepEdge edge in faceAdjacentEdges)
                    foreach (Line l in ll)
                    {
                        Line line = new Line();

                        double distance = 100;
                        int maxAttempts = ll.Length;
                        int counter = 0;

                        while (distance > 1 && counter < maxAttempts)
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

                        edgeDirectionDictionary[edge] = testEdgeVector;
                    }

                foreach (BrepEdge edge in faceAdjacentEdges)
                {
                    BrepVertex bv = edge.StartVertex;
                    Vector3d testVector = edge.PointAtEnd - edge.PointAtStart;

                    if (testVector.IsParallelTo(edgeDirectionDictionary[edge]) > 0)
                        bv = edge.EndVertex;

                    int[] vertexAdjacentEdgesIndexes = bv.EdgeIndices();
                    var vertexAdjacentEdges = body.Edges.Where(e => vertexAdjacentEdgesIndexes.Contains(e.EdgeIndex));

                    var nextBrepEdges = vertexAdjacentEdges.Where(e => e != edge && edgeDirectionDictionary.ContainsKey(e)).ToList();
                    if (nextBrepEdges.Count == 0)
                    {
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Cannot determine next BrepEdge.");
                        continue;
                    }
                    var nextBrepEdge = nextBrepEdges[0];

                    Vector3d va = -edgeDirectionDictionary[edge];
                    Vector3d vb = edgeDirectionDictionary[nextBrepEdge];

                    double angle = Vector3d.VectorAngle(va, vb);
                    Vector3d cp = Vector3d.CrossProduct(va, vb);

                    if (cp.IsParallelTo(faceNormal) == 1)
                        angle = (Math.PI * 2) - angle;

                    vertexIndexes.Add(bv.VertexIndex);
                    anglesDeg.Add(RhinoMath.ToDegrees(angle));
                    anglesRad.Add(angle);

                    vertexPointsForDisplay.Add(bv.Location);
                    vertexIndexesForDisplay.Add(bv.VertexIndex);

                    Arc arc = new Arc(plane: new Plane(bv.Location, vb, -Vector3d.CrossProduct(vb, faceNormal)), center: bv.Location, radius: angulardimensionOffset, angleRadians: angle);

                    dimensionsArcsForDisplay.Add(arc);
                    dimensionsValuesForDisplay.Add(Math.Round(RhinoMath.ToDegrees(angle), 2));
                }

                treevertexIndexes.AddRange(vertexIndexes, new Grasshopper.Kernel.Data.GH_Path(face.FaceIndex));
                treeAnglesRad.AddRange(anglesRad, new Grasshopper.Kernel.Data.GH_Path(face.FaceIndex));
                treeAnglesDeg.AddRange(anglesDeg, new Grasshopper.Kernel.Data.GH_Path(face.FaceIndex));
            }

            DA.SetDataTree(0, treevertexIndexes);
            DA.SetDataTree(1, treeAnglesRad);
            DA.SetDataTree(2, treeAnglesDeg);
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resource1.PlanarAngle;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("8936DF7C-C75B-40AC-A885-D408F2176D36"); }
        }

        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            if (switchVisualise)
            {
                for (int i = 0; i < vertexPointsForDisplay.Count; i++)
                {
                    Plane plane;
                    args.Viewport.GetCameraFrame(out plane);
                    plane.Origin = vertexPointsForDisplay[i];

                    double pixelsPerUnit;
                    args.Viewport.GetWorldToScreenScale(vertexPointsForDisplay[i], out pixelsPerUnit);
                    args.Display.Draw3dText(vertexIndexesForDisplay[i].ToString(), Color.Black, plane, 25 / pixelsPerUnit, "Lucida Console", false, false, TextHorizontalAlignment.Center, TextVerticalAlignment.Middle);
                }
                for (int i = 0; i < facePointsForDisplay.Count; i++)
                {
                    Plane plane;
                    args.Viewport.GetCameraFrame(out plane);
                    plane.Origin = facePointsForDisplay[i];

                    double pixelsPerUnit;
                    args.Viewport.GetWorldToScreenScale(facePointsForDisplay[i], out pixelsPerUnit);
                    args.Display.Draw3dText(faceIndexesForDisplay[i].ToString(), Color.Black, plane, 25 / pixelsPerUnit, "Lucida Console", false, false, TextHorizontalAlignment.Center, TextVerticalAlignment.Middle);
                }
                for (int i = 0; i < dimensionsValuesForDisplay.Count; i++)
                {
                    Plane plane;
                    args.Viewport.GetCameraFrame(out plane);
                    plane.Origin = dimensionsArcsForDisplay[i].PointAt(0.5);

                    double pixelsPerUnit;
                    args.Viewport.GetWorldToScreenScale(dimensionsArcsForDisplay[i].Center, out pixelsPerUnit);
                    args.Display.Draw3dText(dimensionsValuesForDisplay[i].ToString(), Color.Black, plane, 25 / pixelsPerUnit, "Lucida Console", false, false, TextHorizontalAlignment.Center, TextVerticalAlignment.Middle);

                    args.Display.DrawArc(dimensionsArcsForDisplay[i], Color.Aqua);
                }
            }
            //base.DrawViewportMeshes(args);
        }
    }
}