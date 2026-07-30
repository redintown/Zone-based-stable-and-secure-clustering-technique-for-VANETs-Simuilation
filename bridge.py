import socket
import json
import traci
import sys
import os

# Configuration for the TCP Server
HOST = "127.0.0.1"
PORT = 9999  # Must match the port configured in NS-3 SocketClient


def main():
    # Launch SUMO automatically using simulation.sumocfg
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))

        sumo_cfg = os.path.abspath(
            os.path.join(script_dir, "../../sumo/simulation.sumocfg")
        )

        print("Using SUMO config:", sumo_cfg)

        sumoCmd = [
            "sumo-gui",
            "-c",
            sumo_cfg,
            "--start"
        ]

        print("Starting SUMO...")

        traci.start(sumoCmd)

        print("TraCI connected successfully.")

    except Exception as e:
        import traceback
        traceback.print_exc()
        print("Failed to start TraCI/SUMO:", e)
        sys.exit(1)

    # ------------------------------------------------------------------
    # TCP SERVER
    # ------------------------------------------------------------------

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server_socket.bind((HOST, PORT))
    server_socket.listen(1)

    print(f"TCP Server listening on {HOST}:{PORT}. Waiting for NS-3 connection...")

    conn, addr = server_socket.accept()
    print(f"Accepted connection from NS-3 at {addr}")

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            vehicles = []

            for veh_id in traci.vehicle.getIDList():
                x, y = traci.vehicle.getPosition(veh_id)

                vehicles.append({
                    "vehicleId": veh_id,
                    "roadId": traci.vehicle.getRoadID(veh_id),
                    "laneId": traci.vehicle.getLaneID(veh_id),
                    "x": x,
                    "y": y,
                    "speed": traci.vehicle.getSpeed(veh_id),
                    "angle": traci.vehicle.getAngle(veh_id)
                })

            packet = json.dumps({
                "vehicles": vehicles
            }) + "\n"

            conn.sendall(packet.encode("utf-8"))

            data = conn.recv(4096)

            if not data:
                print("NS-3 client disconnected.")
                break

    except KeyboardInterrupt:
        print("Simulation interrupted.")

    except Exception as e:
        import traceback
        traceback.print_exc()
        print("Runtime Error:", e)

    finally:
        print("Cleaning up resources...")

        try:
            conn.close()
        except:
            pass

        try:
            server_socket.close()
        except:
            pass

        try:
            traci.close()
        except:
            pass


if __name__ == "__main__":
    main()
