import numpy as np
import modern_robotics as mr


def FowardDynamics_thetalist(thetalist, dthetalist, t_f, N):
    '''
    This function takes:
        thetalist, dthetalist, and the time step parameters: t_f and N.
    It returns:
        thetalist_matrix-A matrix of joint iterated joint angles
    '''
    
    #Code to Compute the initial "ddthetalist"
    ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, Taulist, g, F_tip, Mlist, Glist, Slist)
    
    dt = t_f / N
    i = 0
    
    # Creates empty matrix list for the joint angles
    thetalist_matrix = []
    
    while i < N:
        # Calculates the next "thetalist", and "dthetalist"
        thetalist, dthetalist = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)
        # Calculates the next "ddthetalist"
        ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, Taulist, g, F_tip, Mlist, Glist, Slist)
        i = i + 1
        #appends the joint angles to the empty matrix
        thetalist_matrix.append(thetalist)
        
    return thetalist_matrix




def write_to_csv(matrix, name):
    '''
    This function takes:
        matrix and;
        name (in string)
    It returns:
        Nothing but;
        write the matrix rows to csv file with the name give
    '''
    import csv
    
    with open(name, "w", newline='') as writefile:
        for row in matrix:
            csv_file = csv.writer(writefile).writerow(row)
    return None
