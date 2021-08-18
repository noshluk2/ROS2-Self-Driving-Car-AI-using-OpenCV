import time
import timeit

import cv2
from req_files.utility import Estimate_MidLane
from req_files import config

def Accumulate(N):
    Total = 0
    for i in range(N + 1):
        Total = Total + i
    #print("Accumalating all Numbers from 1 to ",str(N)," we get =",Total)

def main():
    # Calling function with N=1000 and timing it
    Num =  100000
    start_time = time.time()
    Accumulate(Num)
    end_time = time.time()
    print("\nComparing profiling accuracy of time and timeit \n")
    print("[time]   Accumulate took ",end_time - start_time," sec ")

    # For Benchmarking code [Timeit]
    iterations=100
    print("[Timeit] Accumulate took ",timeit.timeit(lambda:Accumulate(Num), number=iterations)/iterations,"sec")



    # Now Profiling our Midlane_Estimation Algo and see how much time it takes to get the job done
    Midlane = cv2.imread("data/frames/Mid_edge.png",cv2.IMREAD_GRAYSCALE)
    #Estimated_midlane = Estimate_MidLane(Midlane,config.MaxDist_resized)
    #cv2.imshow("Estimated_midlane",Estimated_midlane)
    #cv2.waitKey(0)

    print("\n[Timeit] Estimate_MidLane took ",timeit.timeit(lambda:Estimate_MidLane(Midlane,config.MaxDist_resized), number=iterations)/iterations," sec\n")


if __name__=='__main__':
    main()