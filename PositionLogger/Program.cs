using System.Diagnostics;
using System.Globalization;
using System.Text;
using Valve.VR;

internal static class Program
{
    private const double LogHz = 1.0;
    private const string OutputFileName = "tracking_log.csv";
    private const string FloatFormat = "G9";

    private static int Main()
    {
        EVRInitError initError = EVRInitError.None;
        OpenVR.Init(ref initError, EVRApplicationType.VRApplication_Background);
        if (initError != EVRInitError.None)
        {
            Console.Error.WriteLine($"OpenVR init failed: {initError}");
            return 1;
        }

        var system = OpenVR.System;
        if (system == null)
        {
            Console.Error.WriteLine("OpenVR system not available.");
            OpenVR.Shutdown();
            return 1;
        }

        var outputPath = Path.Combine(AppContext.BaseDirectory, OutputFileName);
        using var writer = new StreamWriter(outputPath, false, new UTF8Encoding(false));
        writer.WriteLine("timestamp_utc,device_index,device_class,controller_role,position_x,position_y,position_z,rotation_w,rotation_x,rotation_y,rotation_z,pose_valid,tracking_result");

        var poses = new TrackedDevicePose_t[(int)OpenVR.k_unMaxTrackedDeviceCount];
        using var quitEvent = new ManualResetEventSlim(false);
        Console.CancelKeyPress += (_, e) =>
        {
            e.Cancel = true;
            quitEvent.Set();
        };

        Console.WriteLine($"Logging at {LogHz} Hz to {outputPath}");
        var interval = TimeSpan.FromSeconds(1.0 / LogHz);
        var stopwatch = Stopwatch.StartNew();
        var nextSample = TimeSpan.Zero;

        while (!quitEvent.IsSet)
        {
            var now = stopwatch.Elapsed;
            if (now < nextSample)
            {
                var delay = nextSample - now;
                if (delay > TimeSpan.Zero)
                {
                    Thread.Sleep(delay);
                }
            }

            if (quitEvent.IsSet)
            {
                break;
            }

            var timestamp = DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture);
            system.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, 0, poses);

            for (uint i = 0; i < poses.Length; i++)
            {
                var pose = poses[i];
                if (!pose.bDeviceIsConnected)
                {
                    continue;
                }

                var deviceClass = system.GetTrackedDeviceClass(i);
                if (deviceClass != ETrackedDeviceClass.HMD && deviceClass != ETrackedDeviceClass.Controller)
                {
                    continue;
                }

                var role = deviceClass == ETrackedDeviceClass.Controller
                    ? system.GetControllerRoleForTrackedDeviceIndex(i).ToString()
                    : "Hmd";

                float px = float.NaN;
                float py = float.NaN;
                float pz = float.NaN;
                float qw = float.NaN;
                float qx = float.NaN;
                float qy = float.NaN;
                float qz = float.NaN;

                if (pose.bPoseIsValid)
                {
                    var m = pose.mDeviceToAbsoluteTracking;
                    px = m.m3;
                    py = m.m7;
                    pz = m.m11;
                    (qw, qx, qy, qz) = QuaternionFromMatrix(m);
                }

                writer.WriteLine(string.Join(",", new[]
                {
                    timestamp,
                    i.ToString(CultureInfo.InvariantCulture),
                    deviceClass.ToString(),
                    role,
                    FormatFloat(px),
                    FormatFloat(py),
                    FormatFloat(pz),
                    FormatFloat(qw),
                    FormatFloat(qx),
                    FormatFloat(qy),
                    FormatFloat(qz),
                    pose.bPoseIsValid ? "true" : "false",
                    pose.eTrackingResult.ToString(),
                }));
            }

            writer.Flush();
            nextSample += interval;
        }

        OpenVR.Shutdown();
        return 0;
    }

    private static (float w, float x, float y, float z) QuaternionFromMatrix(HmdMatrix34_t m)
    {
        var r00 = m.m0;
        var r01 = m.m1;
        var r02 = m.m2;
        var r10 = m.m4;
        var r11 = m.m5;
        var r12 = m.m6;
        var r20 = m.m8;
        var r21 = m.m9;
        var r22 = m.m10;

        var trace = r00 + r11 + r22;
        if (trace > 0.0f)
        {
            var s = MathF.Sqrt(trace + 1.0f) * 2.0f;
            var qw = 0.25f * s;
            var qx = (r21 - r12) / s;
            var qy = (r02 - r20) / s;
            var qz = (r10 - r01) / s;
            return (qw, qx, qy, qz);
        }

        if (r00 > r11 && r00 > r22)
        {
            var s = MathF.Sqrt(1.0f + r00 - r11 - r22) * 2.0f;
            var qw = (r21 - r12) / s;
            var qx = 0.25f * s;
            var qy = (r01 + r10) / s;
            var qz = (r02 + r20) / s;
            return (qw, qx, qy, qz);
        }

        if (r11 > r22)
        {
            var s = MathF.Sqrt(1.0f + r11 - r00 - r22) * 2.0f;
            var qw = (r02 - r20) / s;
            var qx = (r01 + r10) / s;
            var qy = 0.25f * s;
            var qz = (r12 + r21) / s;
            return (qw, qx, qy, qz);
        }

        {
            var s = MathF.Sqrt(1.0f + r22 - r00 - r11) * 2.0f;
            var qw = (r10 - r01) / s;
            var qx = (r02 + r20) / s;
            var qy = (r12 + r21) / s;
            var qz = 0.25f * s;
            return (qw, qx, qy, qz);
        }
    }

    private static string FormatFloat(float value)
    {
        return value.ToString(FloatFormat, CultureInfo.InvariantCulture);
    }
}
