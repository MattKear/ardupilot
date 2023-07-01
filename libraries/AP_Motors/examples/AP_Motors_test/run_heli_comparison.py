# Script that automatically runs multipoint output comparison for all AP_Motors_Heli frames
# To prove equivalence when doing restructuring PR's
#
# Run from "ardupilot" directory
# Run the command below. The first argument is the number of commits to rewind the HEAD to get the "old" 
# comparison point.  This should rewind back past all of the commits you have added
#
# If you just want to run a specific frame class, (e.g. dual heli = 11) then add the -f argument giving the 
# frame class number (e.g. -f 11) to only run that frame class.  Default is to run all heli frame classes
#
# command to run:
# ---------------
# python3 libraries/AP_Motors/examples/AP_Motors_test/run_heli_comparison.py <INESRT No COMMITS TO REWIND>
#
# You may have to run "./waf distclean" if failing to build

import os
import shutil
import csv
from matplotlib import pyplot as plt
from argparse import ArgumentParser

# ==============================================================================
class DataPoints:

    HEADER_LINE = 3

    # --------------------------------------------------------------------------
    # Instantiate the object and parse the data from the provided file
    # file: path to the csv file to be parsed
    def __init__(self, file):
        self.data = {}
        self.limit_case = []

        with open(file, 'r') as csvfile:
            # creating a csv reader object
            csvreader = csv.reader(csvfile)

            # extracting field names through first row
            line_num = 0
            for row in csvreader:
                line_num += 1

                if line_num < self.HEADER_LINE:
                    continue

                elif line_num == self.HEADER_LINE:
                    # init all dict entries based on the header entries
                    for field in row:
                        self.data[field] = []

                else:
                    # stow all of the data
                    case_is_limited = False
                    for field, data in zip(self.data.keys(), row):
                        self.data[field].append(float(data))

                        # Keep track of all cases where a limit flag is set
                        if ('lim' in field.lower()) and (float(data) > 0.5):
                            case_is_limited = True
                    self.limit_case.append(case_is_limited)

            # Make data immutable
            for field in self.data.keys():
                self.data[field] = tuple(self.data[field])
    # --------------------------------------------------------------------------

    # --------------------------------------------------------------------------
    # get the data from a given field
    # field: dict index, name of field data to be returned
    # lim_tf: limit bool, return limit cases or not
    def get_data(self, field, lim_tf):
        if field not in self.data.keys():
            raise Exception('%s is not a valid data field' % field)

        ret = []
        for data, flag in zip(self.data[field], self.limit_case):
            if (flag == lim_tf):
                ret.append(data)
        return ret
    # --------------------------------------------------------------------------

    # --------------------------------------------------------------------------
    def get_fields(self):
        return self.data.keys()
    # --------------------------------------------------------------------------

# ==============================================================================

frame_class_lookup = {6:'Single_Heli', 11:'Dual_Heli', 13:'Heli_Quad'}


# ==============================================================================
if __name__ == '__main__':

    BLUE = [0,0,1]
    RED = [1,0,0]

    dir_name = 'motors_comparison'

    # Build input parser
    parser = ArgumentParser(description='Find logs in which the input string is found in messages')
    parser.add_argument("head", type=int, help='number of commits to roll back the head for comparing the work done')
    parser.add_argument("-f","--frame-class", type=int, dest='frame_class', nargs="+", default=(6,11,13), help="list of frame classes to run comparison on. Defaults to 6 single heli.")
    args = parser.parse_args()

    if not args.head:
        print('Number of commits to roll back HEAD must be provided to run comparison')
        quit()
    if args.head <= 0:
        print('Number of commits to roll back HEAD must be a positive integer value')
        quit()

    # If we have already run this test, delete the old data
    if dir_name in os.listdir('./'):
        shutil.rmtree(dir_name)

    # Create the new directory
    os.mkdir(dir_name)
 
    # configure and build the test
    os.system('./waf configure --board linux')
    os.system('./waf build --target examples/AP_Motors_test')

    print('\nRunning motor tests with current changes\n')

    # run the test
    for fc in args.frame_class:
        filename = 'new_%s_motor_test.csv' % frame_class_lookup[fc]
        os.system('./build/linux/examples/AP_Motors_test s > %s frame_class=%d' % (filename,fc))

        # move the csv to the directory for later comparison
        shutil.move(filename, os.path.join(dir_name, filename))

        print('Frame class = %s complete\n' % frame_class_lookup[fc])

    print('Rewinding head by %d commits\n' % args.head)

    # TEMP: just copying to do plotting for now
    for fc in args.frame_class:
        filename = '%s_motor_test.csv' % frame_class_lookup[fc]
        shutil.copy(os.path.join(dir_name, 'new_%s' % filename), os.path.join(dir_name, 'original_%s' % filename))

    new_points = {}
    old_points = {}

    for fc in args.frame_class:
        filename = '%s_motor_test.csv' % frame_class_lookup[fc]
        new_points[frame_class_lookup[fc]] = DataPoints(os.path.join(dir_name, 'new_%s' % filename))
        old_points[frame_class_lookup[fc]] = DataPoints(os.path.join(dir_name, 'original_%s' % filename))

    # Plot all of the points for correlation comparison
    # Non-limit cases are plotted with blue o's
    # Limited flag cases are plotted with red x's
    for fc in args.frame_class:
        fields = new_points[frame_class_lookup[fc]].get_fields() # Assuming that the fields are the same in all frame classes and between old and new

        # Always start a new plot for a new frame class
        fig, ax = plt.subplots(3, 3, figsize=(13, 13))
        index = [0, 0]
        frame = frame_class_lookup[fc]

        for field in fields:

            max_val = max(max(new_points[frame].data[field]), max(old_points[frame].data[field]))
            min_val = min(min(new_points[frame].data[field]), min(old_points[frame].data[field]))

            ax[index[0],index[1]].plot([min_val, max_val], [min_val, max_val], label='Desired', linestyle='-', marker='')
            ax[index[0],index[1]].plot(new_points[frame].get_data(field, False), old_points[frame].get_data(field, False), label='Not Limited Pts', linestyle='', marker='x', color=BLUE)
            ax[index[0],index[1]].scatter(new_points[frame].get_data(field, True), old_points[frame].get_data(field, True), label='Limited Pts', marker='o', c='None', edgecolors=RED)

            # Set the axis labels on the outter most axes only
            if index[0] == 2:
                ax[index[0],index[1]].set_xlabel('New Points')
            if index[1] == 0:
                ax[index[0],index[1]].set_ylabel('Original Points')

            # Set the title on each subplot
            ax[index[0],index[1]].set_title(field)

            # Move through plot tiles
            index[1] += 1

            if index[1] >= 3:
                index[0] += 1
                index[1] = 0

            if index[0] >= 3:
                index[0] = 0
                index[1] = 0
                # Reached nine tiles so generate new figure
                fig, ax = plt.subplots(3, 3, figsize=(13, 13))


    plt.show()
