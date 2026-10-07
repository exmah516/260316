using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml;
using Kitware.VTK;

namespace Urdf
{

    public struct Point3D
    {
        public double x;
        public double y;
        public double z;

        public void SetValue(String sValue)
        {
            String[] sV = sValue.Split(' ');
            if (3 == sV.Length)
            {
                x = Convert.ToDouble(sV[0]);
                y = Convert.ToDouble(sV[1]);
                z = Convert.ToDouble(sV[2]);
            }
            else
            {
                x = 0; y = 0; z = 0;
            }
        }

        public bool IsZero()
        {
            return 0 == x && 0 == y && 0 == z;
        }
    }

    public class Origin
    {
        public Point3D xyz = new Point3D();
        public Point3D rpy = new Point3D();
    }

    // 节点类型
    public enum NodeType
    {
        Link = 0,                           // 部件
        Joint = 1,                          // 关节
    }

    // 节点基础信息
    public abstract class UrdfNode
    {
        // 节点名字
        public String Name { get; private set; }

        // 节点类型
        public NodeType NType { get; protected set; }

        // 子节点
        //public UrdfNode NextNode { get; set; }
        public List<UrdfNode> NextNodes = new List<UrdfNode>();

        public UrdfNode BeforeNode { get; set; }

        // 设置节点的名称
        public void SetName(XmlNode node)
        {
            XmlElement xe = (XmlElement)node;
            Name = xe.GetAttribute("name").ToString();
        }

        // 读取子节点属性信息
        public String ReadAttribute(XmlNode node, String sChildNodeName, String sAttrName)
        {
            int iPos = sChildNodeName.IndexOf('/');

            String sNodeName = sChildNodeName;
            String sNextChildNodeName = "";

            if (-1 != iPos)
            {
                sNodeName = sChildNodeName.Substring(0, iPos);
                sNextChildNodeName = sChildNodeName.Substring(iPos + 1, sChildNodeName.Length - iPos - 1);
            }

            foreach (XmlNode n in node)
            {
                if (sNodeName == n.Name)
                {
                    if ("" == sNextChildNodeName)
                    {
                        return ((XmlElement)n).GetAttribute(sAttrName);
                    }
                    else
                    {
                        return ReadAttribute(n, sNextChildNodeName, sAttrName);
                    }
                }
            }

            return "";
        }

        public abstract bool Load(XmlNode node);
    }

    // link节点
    public class UrdfLink : UrdfNode
    {
        // 惯性信息 inertial节点
        class Inertial
        {
            public Origin mOrigin = new Origin();
            //public Point3D mOrigin = new Point3D();
            //public Point3D mRpy = new Point3D();
            public double mMass;                   // 重量信息
            public Point3D mix = new Point3D();
            public Point3D miy = new Point3D();
        }
        Inertial mInertial = new Inertial();

        enum ModelType
        {
            NONE = 0,                   // 未知模型
            FILE = 1,                   // 文件模型
            SPHERE = 2,                 // 圆球模型
            BOX = 3,                    // 立方体
            CYLINDER = 4,               // 圆柱体
        }

        // 视觉显示信息
        class Visual
        {
            public Origin mOrigin = new Origin();
            //public Point3D mOrigin = new Point3D();
            //public Point3D mRpy = new Point3D();

            public ModelType mMType = ModelType.NONE;
            // 模型
            public String mFileName = "";
            public double dR = 0;

            public double r, g, b, a;

            // 显示的actor
            public vtkActor actor = vtkActor.New();

        }

        Visual mVisual = new Visual();

        bool bIsInitPosOver = false;

        // 碰撞信息  暂时不处理
        //??

        public override bool Load(XmlNode node)
        {
            SetName(node);

            // 部件节点
            NType = NodeType.Link;

            // 读取节点数据
            foreach (XmlNode n in node)
            {
                XmlElement nXe = (XmlElement)n;
                if ("inertial" == n.Name)
                {
                    // 惯性信息
                    mInertial.mOrigin.xyz.SetValue(ReadAttribute(n, "origin", "xyz"));
                    mInertial.mOrigin.rpy.SetValue(ReadAttribute(n, "origin", "rpy"));
                    mInertial.mMass = Convert.ToDouble(ReadAttribute(n, "mass", "value"));

                    mInertial.mix.x = Convert.ToDouble(ReadAttribute(n, "inertia", "ixx"));
                    mInertial.mix.y = Convert.ToDouble(ReadAttribute(n, "inertia", "ixy"));
                    mInertial.mix.z = Convert.ToDouble(ReadAttribute(n, "inertia", "ixz"));

                    mInertial.miy.x = Convert.ToDouble(ReadAttribute(n, "inertia", "iyy"));
                    mInertial.miy.y = Convert.ToDouble(ReadAttribute(n, "inertia", "iyz"));
                    mInertial.miy.z = Convert.ToDouble(ReadAttribute(n, "inertia", "izz"));

                }
                else if ("visual" == n.Name)
                {
                    // 视觉显示信息
                    mVisual.mOrigin.xyz.SetValue(ReadAttribute(n, "origin", "xyz"));
                    mVisual.mOrigin.rpy.SetValue(ReadAttribute(n, "origin", "rpy"));

                    mVisual.mFileName = ReadAttribute(n, "geometry/mesh", "filename");
                    mVisual.mMType = ModelType.FILE;
                    if ("" == mVisual.mFileName)
                    {
                        String sR = ReadAttribute(n, "geometry/sphere", "radius");
                        mVisual.dR = Convert.ToDouble(sR);
                        mVisual.mMType = ModelType.SPHERE;
                    }

                    String sColor = ReadAttribute(n, "material/color", "rgba");
                    String[] sList = sColor.Split(' ');

                    mVisual.r = Convert.ToDouble(sList[0]);
                    mVisual.g = Convert.ToDouble(sList[1]);
                    mVisual.b = Convert.ToDouble(sList[2]);
                    mVisual.a = Convert.ToDouble(sList[3]);

                }
                else if ("collision" == n.Name)
                {
                    // 碰撞检测信息

                }
            }

            // XmlElement xe = (XmlElement)node;
            //node.ChildNodes()

            return true;
        }

        // 添加模型
        public bool AddModel(String sBasePath, vtkRenderer ren)
        {
            if (ModelType.FILE != mVisual.mMType) return true;

            //if ("sl0" != Name && "base_link" != Name && "sl1" != Name
            //     && "sl2" != Name && "sl3" != Name && "sl4" != Name && "sl5" != Name && "sl6" != Name
            //     && "sl7" != Name) return true;
            ////if ("base_link" != Name && "sl4" != Name) return true;

            // 添加模型
            vtkSTLReader vtkSTLReader = new vtkSTLReader();
            int iPos = mVisual.mFileName.IndexOf("//");
            vtkSTLReader.SetFileName(System.IO.Path.Combine(sBasePath, mVisual.mFileName.Substring(iPos + 2)));

            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            mapper.SetInputConnection(vtkSTLReader.GetOutputPort());
            // Link the data pipeline to the rendering subsystem
            mVisual.actor.SetMapper(mapper);
            mVisual.actor.GetProperty().SetColor(mVisual.r, mVisual.g, mVisual.b);

            //actor.SetPosition()

            //if ( "sl4" == Name ) {
            //   mVisual.actor.SetPosition(-0.094533, 0.29224, 0.54912);
            //}
            //            actor.RotateX

            ren.AddActor(mVisual.actor);

            return true;
        }

        //// 初始化位置
        //public void ModelInitPos(Origin ori) {
        //    mVisual.actor.SetPosition(ori.xyz.x, ori.xyz.y, ori.xyz.z);
        //    //for (int i = 0; i < vOri.Count; i++) {
        //    //}
        //}

        // 初始化位置
        public void ModelInitPos(List<Origin> vOri)
        {
            if (bIsInitPosOver) return;
            bIsInitPosOver = true;

            double[] dM = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

            double x = 0, y = 0, z = 0;
            for (int i = 0; i < vOri.Count; i++)
            {
                // 计算在世界坐标系下的映射坐标
                // unity
                x += vOri[i].xyz.x * dM[0] + vOri[i].xyz.y * dM[1] + vOri[i].xyz.z * dM[2];
                y += vOri[i].xyz.x * dM[4] + vOri[i].xyz.y * dM[5] + vOri[i].xyz.z * dM[6];
                z += vOri[i].xyz.x * dM[8] + vOri[i].xyz.y * dM[9] + vOri[i].xyz.z * dM[10];

                // 计算旋转矩阵
                UrdfMath.GetMatrixByAngle(vOri[i].rpy.x, vOri[i].rpy.y, vOri[i].rpy.z, out double[] m);
                UrdfMath.GetMatrixByAngle2(ref dM, ref m);
            }

            // 设置旋转矩阵与位置
            vtkMatrix4x4 mm = vtkMatrix4x4.New();

            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    mm.SetElement(i, j, dM[i * 4 + j]);
                }
            }

            mm.SetElement(0, 3, x);
            mm.SetElement(1, 3, y);
            mm.SetElement(2, 3, z);

            mVisual.actor.SetUserMatrix(mm);
        }

        // 模型移动
        public void ModelMove(List<UrdfJoint> vJoits)
        {
            double[] dM = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

            double x = 0, y = 0, z = 0;
            for (int i = 0; i < vJoits.Count; i++)
            {
                Origin ori = vJoits[i].GetOrigin();
                // 计算在世界坐标系下的映射坐标
                // unity
                x += ori.xyz.x * dM[0] + ori.xyz.y * dM[1] + ori.xyz.z * dM[2];
                y += ori.xyz.x * dM[4] + ori.xyz.y * dM[5] + ori.xyz.z * dM[6];
                z += ori.xyz.x * dM[8] + ori.xyz.y * dM[9] + ori.xyz.z * dM[10];

                // 计算旋转矩阵
                UrdfMath.GetMatrixByAngle(ori.rpy.x, ori.rpy.y, ori.rpy.z, out double[] m);

                if (vJoits[i].IsAngle())
                {
                    // 旋转矩阵
                    vJoits[i].GetAngleMatrix(out double[] dMatrix);
                    UrdfMath.GetMatrixByAngle2(ref m, ref dMatrix);
                    UrdfMath.GetMatrixByAngle2(ref dM, ref m);
                }
                else
                {
                    UrdfMath.GetMatrixByAngle2(ref dM, ref m);
                    vJoits[i].GetMoveXYZ(dM, ref x, ref y, ref z);
                }
            }

            // 设置旋转矩阵与位置
            vtkMatrix4x4 mm = vtkMatrix4x4.New();

            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    mm.SetElement(i, j, dM[i * 4 + j]);
                }
            }

            mm.SetElement(0, 3, x);
            mm.SetElement(1, 3, y);
            mm.SetElement(2, 3, z);

            mVisual.actor.SetUserMatrix(mm);

        }
    }

    // joint节点
    public class UrdfJoint : UrdfNode
    {
        //enum JointType {
        //    FIXED,                      // 固定，不是真正的关节，不能移动
        //    Revolute,                   // 沿轴旋转的铰链关节，即转动关节，具有由上限和下限指定的有限范围
        //    Continuous,                 // (旋转) 绕轴旋转的连续铰链关节，即转动关节，没有上限和下限。
        //    Prismatic,                  // (平移) 沿轴滑动的滑动关节，即平动关节，具有由上限和下限指定的有限范围。
        //    Floating,                   // 浮动关节，此关节允许所有6个自由度的运动。
        //    Planar,                     // 平面关节，该关节允许在垂直于轴线的平面内运动。
        //}
        //JointType mJointType = JointType.FIXED;
        String msJointType = "";

        Origin mOrigin = new Origin();
        //        Point3D mOrigin = new Point3D();
        //        Point3D mRpy = new Point3D();

        String msParent = "";
        String msChild = "";

        // 旋转轴,移动方向
        Point3D mAxis = new Point3D();
        // 旋转角度或移动距离
        double mdMove = 0;

        // limit
        double lower, upper, effort, velocity;

        public override bool Load(XmlNode node)
        {
            SetName(node);

            // 部件节点
            NType = NodeType.Joint;

            msJointType = ((XmlElement)node).GetAttribute("type").ToString();
            mOrigin.xyz.SetValue(ReadAttribute(node, "origin", "xyz"));
            mOrigin.rpy.SetValue(ReadAttribute(node, "origin", "rpy"));
            msParent = ReadAttribute(node, "parent", "link");
            msChild = ReadAttribute(node, "child", "link");
            mAxis.SetValue(ReadAttribute(node, "axis", "xyz"));

            if ("prismatic" == msJointType)
            {
                lower = Convert.ToDouble(ReadAttribute(node, "limit", "lower"));
                upper = Convert.ToDouble(ReadAttribute(node, "limit", "upper"));
                effort = Convert.ToDouble(ReadAttribute(node, "limit", "effort"));
                velocity = Convert.ToDouble(ReadAttribute(node, "limit", "velocity"));
            }

            return true;
        }

        public String ParentName() { return msParent; }

        public String ChildName() { return msChild; }

        public Origin GetOrigin() { return mOrigin; }

        public void SetMove(double dMove) { mdMove = dMove; }

        public void GetAngleMatrix(out double[] dMatrix)
        {
            UrdfMath.GetMatrixByAngle(mAxis.x * mdMove, mAxis.y * mdMove, mAxis.z * mdMove, out dMatrix);
        }

        public void GetMoveXYZ(double[] dM, ref double dX, ref double dY, ref double dZ)
        {
            dX += mdMove * mAxis.x * dM[0] + mdMove * mAxis.y * dM[1] + mdMove * mAxis.z * dM[2];
            dY += mdMove * mAxis.x * dM[4] + mdMove * mAxis.y * dM[5] + mdMove * mAxis.z * dM[6];
            dZ += mdMove * mAxis.x * dM[8] + mdMove * mAxis.y * dM[9] + mdMove * mAxis.z * dM[10];
        }

        public bool IsAngle()
        {
            return msJointType.StartsWith("continuous");
        }
    }


    // URDF数据保存
    public class UrdfData
    {
        private String mModelName = "";

        // Link 列表      部件列表
        private List<UrdfLink> mLinks = new List<UrdfLink>();

        // Joint 列表     连接关节列表
        private List<UrdfJoint> mJoints = new List<UrdfJoint>();

        // head 列表      连接头部列表
        private List<UrdfNode> mHeads = new List<UrdfNode>();

        // 文件读取
        public bool Load(String sFile)
        {
            XmlDocument doc = new XmlDocument();
            doc.Load(sFile);
            XmlNode nodeRoot = doc.DocumentElement;

            mModelName = ((XmlElement)nodeRoot).GetAttribute("name").ToString();

            // 读取全部 部件与连接列表
            mLinks.Clear();
            mJoints.Clear();
            // 获取每个子节点数据
            foreach (XmlNode node in nodeRoot)
            {
                // MessageBox.Show(node.Name);
                if ("link" == node.Name)
                {
                    UrdfLink link = new UrdfLink();
                    link.Load(node);
                    mLinks.Add(link);
                }
                else if ("joint" == node.Name)
                {
                    UrdfJoint joint = new UrdfJoint();
                    joint.Load(node);
                    mJoints.Add(joint);
                }
            }

            // 将部件连接为整体
            for (int i = 0; i < mJoints.Count; i++)
            {
                for (int j = 0; j < mLinks.Count; j++)
                {
                    // 未处理多子节点
                    if (mJoints[i].ParentName() == mLinks[j].Name)
                    {
                        mLinks[j].NextNodes.Add(mJoints[i]);
                        mJoints[i].BeforeNode = mLinks[j];
                    }
                    else if (mJoints[i].ChildName() == mLinks[j].Name)
                    {
                        mJoints[i].NextNodes.Add(mLinks[j]);
                        mLinks[j].BeforeNode = mJoints[i];
                    }
                }
            }

            return true;
        }

        // 添加模型文件
        public bool AddModel(String sBasePath, vtkRenderer ren)
        {
            // 添加模型
            for (int i = 0; i < mLinks.Count; i++)
            {
                mLinks[i].AddModel(sBasePath, ren);
            }

            return true;
        }

        // 初始化关键链表
        public bool Init()
        {
            // 初始化头部
            for (int i = 0; i < mLinks.Count; i++)
            {
                if (null == mLinks[i].BeforeNode)
                    mHeads.Add(mLinks[i]);
            }
            UpdateMove();

            return true;
        }

        // 获取关节列表
        public List<UrdfJoint> GetUrdfJoints()
        {
            return mJoints;
        }

        // 关节旋转或移动
        public void MoveJoint(String sJointName, double dMoveValue)
        {
            for (int i = 0; i < mJoints.Count; i++)
            {
                if (sJointName == mJoints[i].Name)
                {
                    mJoints[i].SetMove(dMoveValue);
                    break;
                }
            }
        }

        // 更新移动位置
        public void UpdateMove()
        {
            // 根据关节信息，更新部件位置。
            List<UrdfJoint> vJoint = new List<UrdfJoint>();

            for (int i = 0; i < mHeads.Count; i++)
            {
                vJoint.Clear();
                // 初始化位置
                Move(mHeads[i], vJoint);
            }
        }



        // 初始化位置角度， 按照连接递归调用
        private void InitPos(UrdfNode node, List<Origin> vOrigin)
        {
            // 到结尾了
            if (null == node) return;

            if (node.NType == NodeType.Joint)
            {
                // 关节节点
                Origin ori = (node as UrdfJoint).GetOrigin();//.xyz
                if (!ori.xyz.IsZero() || !ori.rpy.IsZero())
                {
                    vOrigin.Add(ori);
                }
            }
            else if (node.NType == NodeType.Link)
            {
                if (vOrigin.Count > 0)
                {
                    (node as UrdfLink).ModelInitPos(vOrigin);
                }
            }

            // 继续初始化下一个节点
            for (int i = 0; i < node.NextNodes.Count; i++)
            {
                InitPos(node.NextNodes[i], vOrigin);
            }
        }

        // 移动指定关节
        private void Move(UrdfNode node, List<UrdfJoint> vJoints)
        {
            if (null == node) return;
            if (node.NType == NodeType.Joint)
            {
                // 关节节点
                //Origin ori = (node as UrdfJoint).GetOrigin();//.xyz
                //                if (!ori.xyz.IsZero() || !ori.rpy.IsZero()) {
                vJoints.Add((node as UrdfJoint));
                //                }
            }
            else if (node.NType == NodeType.Link)
            {
                if (vJoints.Count > 0)
                {
                    (node as UrdfLink).ModelMove(vJoints);
                }
            }

            // 继续初始化下一个节点
            for (int i = 0; i < node.NextNodes.Count; i++)
            {
                List<UrdfJoint> vJ = new List<UrdfJoint>(vJoints);
                Move(node.NextNodes[i], vJ);
            }
        }

    }
}
