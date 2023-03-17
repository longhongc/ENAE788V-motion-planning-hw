./solution 1 30 -35 1.57 0 0 5 10
./solution 2 40 -40 0 30 40 5 5
./solution 3 30 40 4.71 -50 -30 5 5
./solution 4 -50 -30 1.57 30 -35 5 3
./solution 5 -30 -35 -0.785 -35 30 5 2

echo "Creating rrt graph ..."
python3 draw_rrt.py 1 30 -35 1.57 0 0 5
python3 draw_rrt.py 2 40 -40 0 30 40 5
python3 draw_rrt.py 3 30 40 4.71 -50 -30 5
python3 draw_rrt.py 4 -50 -30 1.57 30 -35 5
python3 draw_rrt.py 5 -30 -35 -0.785 -35 30 5
