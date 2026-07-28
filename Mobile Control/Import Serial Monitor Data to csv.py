import serial
import csv
import time

# Serial setup
ser = serial.Serial('COM14', 115200, timeout=1)
time.sleep(2)

# Open CSV file
with open('stwpid.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    
    # Correct header
    writer.writerow(["Time", "Yaw", "Encoder Left", "Encoder Right"])

    print("Recording started... Press Ctrl+C to stop")

    try:
        while True:
            if ser.in_waiting > 0:
                try:
                    data = ser.readline().decode('utf-8').strip()
                    print(data)

                    values = data.split(',')

                    # Expecting exactly 4 values
                    if len(values) == 4:
                        # Convert to proper types (important)
                        t = float(values[0])
                        yaw = float(values[1])
                        left = int(values[2])
                        right = int(values[3])

                        writer.writerow([t, yaw, left, right])
                    else:
                        print("Invalid data:", data)

                except ValueError:
                    # Handles corrupted or partial data
                    print("Parsing error:", data)

    except KeyboardInterrupt:
        print("\nRecording stopped")
        ser.close()