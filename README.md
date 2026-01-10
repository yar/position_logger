# Position Logger

Open the project file in Visual Studio 2026.
Switch to release and build.


Make sure SteamVR is ready. Run the logger without parameters:

PositionLogger\bin\Release\net8.0\PositionLogger.exe

It will start recording positions and orientations of the headset and the controllers at 90 samples per second. Move the headset and the controllers to produce enough data.

Press Ctrl-C when finished.

The CSV file will be saved as PositionLogger\bin\Release\net8.0\tracking_log.csv

Logging frequency can be changed: edit `LogHz` in `PositionLogger/Program.cs`.
