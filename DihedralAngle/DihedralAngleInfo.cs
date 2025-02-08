using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace DihedralAngle
{
    public class DihedralAngleInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "DihedralAngle";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("f27d9d1d-2256-45df-a002-12dd7722cc58");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
