using System;
using System.Runtime.InteropServices;

namespace MasterConsole.Services
{
    [StructLayout(LayoutKind.Sequential)]
    public struct CCFrame
    {
        public byte type;
        public byte length;
        public UInt32 id;
        public UInt32 timestamp;

        [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.U1, SizeConst = 8)]
        public byte[] data;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct control_cmd
    {
        public double time;

        public double wire_pos;
        public double pipe_pos;
        public double balloon_pos;
        public double rail_pos;
        public double wire_angle;
        public double pipe_angle;

        public byte btn_wire1;
        public byte btn_wire2;
        public byte btn_pipe1;
        public byte btn_pipe2;
        public byte btn_inspire;

        public CCFrame ccF;
    }
}
