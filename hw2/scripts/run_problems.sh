./solution 1 0 0 -38 20 10 10
./solution 2 27 30 -48 20 10 5
./solution 3 45 -45 -45 45 15 5
./solution 4 -16 10 18 -45 5 2
./solution 5 39 5 -38 -8 3 1

echo "Creating rrt graph ..."
python3 draw_rrt.py 1 0 0 -38 20 10
python3 draw_rrt.py 2 27 30 -48 20 10
python3 draw_rrt.py 3 45 -45 -45 45 15
python3 draw_rrt.py 4 -16 10 18 -45 5
python3 draw_rrt.py 5 39 5 -38 -8 3
