import math
import serial

try:
    ser = serial.Serial('/dev/cu.usbserial-1420', 9600)
except serial.SerialException as e:
    print(f"Serial connection failed: {e}")
    ser = None

segmentLength1 = 140
segmentLength2 = 140
segmentLength3 = 140

xCord = float(input("enter x: "))
yCord = float(input("enter y: "))
targetDir = math.radians(float(input("enter target direction: ")))

J3x = xCord - segmentLength3 * math.cos(targetDir)
J3y = yCord - segmentLength3 * math.sin(targetDir)

D2 = math.hypot(J3x, J3y)
distanceAngle = math.atan2(J3y, J3x)

if D2 > segmentLength1 + segmentLength2 or D2 < abs(segmentLength1 - segmentLength2):
    print("unreachable")
else:
    val = max(-1, min(1, (39200 - D2**2) / 39200))
    alpha2 = math.acos(val)
    alpha1 = (math.pi - alpha2) / 2

    def to_signed_180(deg):
        """Convert any angle to the range (-180, 180]."""
        deg = deg % 360
        if deg > 180:
            deg -= 360
        return deg

    def global_to_local(abs_angle_rad, parent_abs_angle_rad):
        straight_up = parent_abs_angle_rad + math.pi / 2
        local = straight_up - abs_angle_rad
        return to_signed_180(math.degrees(local))

    def compute_joints(use_elbow_up):
        sign = 1 if use_elbow_up else -1

        seg1_abs = distanceAngle + sign * alpha1
        seg2_abs = seg1_abs - sign * (math.pi - alpha2)
        seg3_abs = math.atan2(yCord - J3y, xCord - J3x)

        J1 = to_signed_180(math.degrees(math.pi / 2 - seg1_abs))
        J2 = global_to_local(seg2_abs, seg1_abs)
        J3 = global_to_local(seg3_abs, seg2_abs)

        return J1, J2, J3

    def reconstruct_positions(j1_deg, j2_deg, j3_deg):
        seg1_abs = math.pi / 2 - math.radians(j1_deg)
        seg2_abs = (seg1_abs + math.pi / 2) - math.radians(j2_deg)
        seg3_abs = (seg2_abs + math.pi / 2) - math.radians(j3_deg)

        p0 = (0, 0)
        p1 = (p0[0] + segmentLength1 * math.cos(seg1_abs),
               p0[1] + segmentLength1 * math.sin(seg1_abs))
        p2 = (p1[0] + segmentLength2 * math.cos(seg2_abs),
               p1[1] + segmentLength2 * math.sin(seg2_abs))
        p3 = (p2[0] + segmentLength3 * math.cos(seg3_abs),
               p2[1] + segmentLength3 * math.sin(seg3_abs))

        return p1, p2, p3

    def is_valid(j1, j2, j3):
        # Reject if any angle exceeds ±180
        if abs(j1) > 180 or abs(j2) > 180 or abs(j3) > 180:
            return False
        p1, p2, p3 = reconstruct_positions(j1, j2, j3)
        if p1[1] < 0 or p2[1] < 0 or p3[1] < 0:
            return False
        return True

    up = compute_joints(use_elbow_up=True)
    down = compute_joints(use_elbow_up=False)

    if is_valid(*up):
        J1, J2, J3 = up
        print("Using elbow-up solution")
    elif is_valid(*down):
        J1, J2, J3 = down
        print("Using elbow-down solution")
    else:
        print("unreachable: no valid solution avoids self-collision or joint limits")
        J1 = J2 = J3 = None

    if J1 is not None:
        print("J1:", round(J1, 2))
        print("J2:", round(J2, 2))
        print("J3:", round(J3, 2))

        if ser is not None:
            try:
                message = f"{round(J1, 2)},{round(J2, 2)},{round(J3, 2)}\n"
                ser.write(message.encode())
            except serial.SerialException as e:
                print(f"Failed to send to serial: {e}")
        else:
            print("Serial not connected, skipping send.")

    p1, p2, p3 = reconstruct_positions(J1, J2, J3)
    print("end effector landed at:", round(p3[0], 2), round(p3[1], 2))
    print("target was:", xCord, yCord)
