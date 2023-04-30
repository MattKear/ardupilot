# Script that automatically runs multipoint output comparison for all AP_Motors_Heli frames
# To prove equivalence when doing restructuring PR's

# Run from ardupilot directory



import os
import shutil
import csv
from matplotlib import pyplot as plt


class DataPoints:

    HEADER_LINE = 6

    def __init__(self, file):
        self.data = {}

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
                    for field, data in zip(self.data.keys(), row):
                        self.data[field].append(float(data))







if __name__ == '__main__':

    dir_name = 'motors_comparison'
    compare_file = 'motors_test.csv'

    # If we have already run this test, delete the old data
    if dir_name in os.listdir('./'):
        shutil.rmtree(dir_name)

    # Create the new directory
    os.mkdir(dir_name)
 
    # configure and build the test
    os.system('./waf configure --board linux')
    os.system('./waf build --target examples/AP_Motors_test')

    # run the test
    print('\nRunning motor test with current changes\n')
    os.system('./build/linux/examples/AP_Motors_test s > new_%s' % compare_file)

    # move the csv to the directory for later comparison
    shutil.move('new_%s' % compare_file, os.path.join(dir_name, 'new_%s' % compare_file))

    print('Run Complete\n')

    print('Rewinding head by x commits\n')

    # TEMP: just copying to do plotting for now
    shutil.copy(os.path.join(dir_name, 'new_%s' % compare_file), os.path.join(dir_name, 'original_%s' % compare_file))

    new_points = DataPoints(os.path.join(dir_name, 'new_%s' % compare_file))

    old_points = DataPoints(os.path.join(dir_name, 'original_%s' % compare_file))


    for field in new_points.data.keys():
        max_val = max(max(new_points.data[field]), max(old_points.data[field]))
        min_val = min(min(new_points.data[field]), min(old_points.data[field]))

        fig, ax = plt.subplots()
        ax.plot([min_val, max_val], [min_val, max_val], label='Desired', linestyle='-', marker='')
        ax.plot(new_points.data[field], old_points.data[field], label='Data Pts', linestyle='', marker='x')
        ax.set_xlabel('New Points')
        ax.set_ylabel('Original Points')
        ax.set_title(field)


    plt.show()









