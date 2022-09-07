FILE=../../../problems/drones/flight_test/log  
FILE=${FILE%}_`date +"%Y-%m-%d_%H-%M-%S"`.txt
# echo $FILE                                 

python3 multidrone_flight_test.py 2 4 2>&1 | tee -a $FILE
