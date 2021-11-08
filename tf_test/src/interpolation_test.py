import csv
import numpy as np
import copy
import random

lookup_list = []
lookup_table = []
class Interpolate_welding:
    
    def __init__(self, filename):
        self.csvfile = open(filename, 'r')
        self.lookup_table = []
        self.lookup_list = []
        make_lookup_list()
        return

    def make_lookup_list(self):
        lookup_table.clear()
        lookup_list.clear()
        rdr = csv.reader(self.csvfile)
        for line in rdr:
            self.lookup_list.append(line)
        return
    def make_lookup_table(self, num_points):
        random.shuffle(lookup_list)
        for i in range(0,point_num):
            key_point = np.array(float(lookup_list[i][1]), float(lookup_list[i][2]), float(lookup_list[i][3])
            err_trans = np.array(float(lookup_list[i][4]), float(lookup_list[i][5]), float(lookup_list[i][6])
            lookup_table.append([key_point, err_trans])
        return

    def interpolate_point(self,point_cor, method):
        if len(self.lookup_table) == 0:
            return np.array(0,0,0)
        distance_list = []
        for point, err in lookup_table:
            distance_list.append([np.linalg.norm(point-point_cor),err])
        if method == 'nearest':
            _ , nearest_err = distance_list[distance_list.index(min(distance_list))]
            return nearest_err
        elif method == 'linear':
            linear_interp_list = []
            for i in range(0,4):
                linear_interp_list.append(distance_list.pop(distance_list.index(min(distance_list))))
            s_distance= sum(linear_interp_list)
            linear_err = np.array(0,0,0)
            for distance, trans_err in linear_interp_list:
                linear_err += (1/distance * trans_err)
            linear_err /= s_distance
            return linear_err
    
    return result
def main():
    print('enter lookup csv filename'))
    l_filename = raw_input()
    l_filename += '.csv'
    interp_module = Interpolate_welding(l_filename)
    print('select interpolate point number')
    point_number = input()
    if int(point_number) > len(interp_module.lookup_list):
        print("it's too big number to interpolte")
        return  
    interp_module.make_lookup_table(int(point_number))
    print('enter test csvfile')
    test_csvfilename = raw_input()
    test_csvfilename += '.csv'
    t_csvfile = open(test_csvfilename,'r')
    t_rdr = csv.reader(t_csvfile)
    for line in t_rdr:
        x, y, z = (float(line[1]), float(line[2]), float(line[3]))
        
    
    return
if __name__ == '__main__':
    main()