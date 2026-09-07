// IBicycleInput.cs
public interface IBicycleInput
{
    float GetSteerInput();   // -1 (left) to 1 (right)
    float GetDriveInput();   // 0 to 1
    float GetBrakeInput();   // 0 to 1
}