''' test_set_2.py

    Demo script to train and test a point-based classifier for dataset 2
    
    Folders should be organized:
    src / test_set_2.py  (+ other python files)
    data / set_2_train.csv (+ other data files)

'''
from pathlib import Path
from labeled_data import LabeledData
from classifier import Classifier
import numpy as np

def add_data_channels(data_in: LabeledData) -> LabeledData:
    x = data_in.data[:, 0]
    y = data_in.data[:, 1]

    x_squared = (x * x).reshape(-1, 1)
    y_squared = (y * y).reshape(-1, 1)
    x_times_y = (x * y).reshape(-1, 1)

    new_data = np.hstack((data_in.data, x_squared, y_squared, x_times_y))
    
    new_labeled_data = LabeledData()
    new_labeled_data.data = new_data
    new_labeled_data.labels = data_in.labels
    return new_labeled_data

path = Path(__file__).parents[1] / 'data'  # Get path to data folder

train = LabeledData( str( path / 'set_2_train.csv' ) )  # Load train data
test = LabeledData( str( path / 'set_2_test.csv' ) )    # Load test data

train_plus = add_data_channels(train)
test_plus = add_data_channels(test)

model = Classifier()

# Fit classifier parameters to training data:
model.fit(train_plus)

# Plot target and clutter points from test set:
model.plot_all_points(test, fignum='Input_2', title='Test Data 2', block=False)

# Classify test points:
scores = model.classify(test_plus)
print(f'Average Precision: {model.average_precision(test_plus.labels, scores):.3f}')

# Plot classification results:
model.plot_results(test, scores, fignum='Result_2_Plus', block=False)
model.plot_hist(test_plus.labels, scores, fignum='Hist_2_Plus', block=False)

model.plot_pr_curve(test_plus.labels, scores, fignum="PR_Curve", block=True, filesave='../ex1/pr_set_2_plus.png')