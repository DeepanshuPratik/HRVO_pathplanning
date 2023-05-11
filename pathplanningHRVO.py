import reachtarget as HRVO
import argparse

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-G1x", "--Goal1_x", help="Robot1 goal x-coordinate")
    parser.add_argument(
        "-G1y", "--Goal1_y", help="Robot1 goal y-coordinate")
    parser.add_argument(
        "-G2x", "--Goal2_x", help="Robot2 Goal x-coordinate")
    parser.add_argument(
        "-G2y", "--Goal2_y", help="Robot2 Goal y-coordinate")
    parser.add_argument(
        "-Oc", "--Obstacle_count", help="Number of obstacles")
    parser.add_argument(
        "-Rr", "--Robot_radius", help="Robot radius"
    )

    args = parser.parse_args()
    
    HRVO.simulate(args.Goal1_x,args.Goal1_y,args.Goal2_x,args.Goal2_y,args.Obstacle_count,args.Robot_radius)
