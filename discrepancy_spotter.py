import os

root_dir = '/data/cmpe297-03-sp19/PilotA/Argoverse_3d_tracking/argoverse-tracking/'
disparity_dir = root_dir + 'disparity/'
stereo_left_dir = root_dir + 'stereo_left/'
stereo_right_dir = root_dir + 'stereo_right/'

disp_list = os.listdir(disparity_dir)
left_list = os.listdir(stereo_left_dir)
right_list = os.listdir(stereo_right_dir)

only_time_disp = []
for disp in disp_list:
    disp_split = disp.split('.')
    only_time_disp.append(disp_split[0][:10])

only_time_left = []
for left in left_list:
    left_split = left.split('.')
    left_split = left_split[0]
    left_split = left_split.split('_')
    only_time_left.append(left_split[3][:10])

only_time_right = []
for right in right_list:
    right_split = right.split('.')
    right_split = right_split[0]
    right_split = right_split.split('_')
    only_time_right.append(right_split[3][:10])

for i in range(len(only_time_left)):
    current_left = only_time_left[i]
    if(current_left not in only_time_disp):
        print(current_left)

for i in range(len(only_time_disp)):
    current_disp = only_time_disp[i]
    if(current_disp not in only_time_left):
        print("print = " + current_disp)