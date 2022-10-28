FILE=../../../problems/drones/on-screen-logs/on_screen_log  
FILE=${FILE%}_`date +"%Y-%m-%d_%H-%M-%S"`.txt
# echo $FILE                                 

make -j8 2>&1 && ./solve 2>&1 | tee -a $FILE && ./simulate 2>&1 | tee -a $FILE 
