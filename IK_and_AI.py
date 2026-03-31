import math
import serial
ser = serial.Serial('/dev/cu.usbserial-1420', 9600)

segmentLength1 = 140
segmentLength2 = 140
segmentLength3 = 140

xCord = float(input("enter x: "))
yCord = float(input("enter y: "))
targetDir = math.radians(float(input("enter target direction: ")))

J3x = xCord - segmentLength3 * math.cos(targetDir)
J3y = yCord - segmentLength3 * math.sin(targetDir)

distance = math.hypot(xCord, yCord)
distanceAngle = math.atan2(J3y, J3x)
D2 = math.hypot(J3x, J3y)

if D2 > segmentLength1 + segmentLength2 or D2 < abs(segmentLength1 - segmentLength2) or distance > segmentLength1 + segmentLength2 + segmentLength3:
    print("unreachable")
else:
    val = max(-1, min(1, (39200 - D2**2) / 39200))
    alpha2 = math.acos(val)
    alpha1 = (math.pi - alpha2) / 2

    J1 = math.degrees(distanceAngle + alpha1)
    J2 = math.degrees(alpha2) + 180
    J3 = (math.degrees(math.atan2(yCord - J3y, xCord - J3x)) - J1 + math.degrees(alpha2)) - 90

    print("J1:", round(J1, 2))
    print("J2:", round(J2, 2))
    print("J3:", round(J3, 2))

    message = f"{round(J1, 2)},{round(J2, 2)},{round(J3, 2)}\n"
    ser.write(message.encode())
