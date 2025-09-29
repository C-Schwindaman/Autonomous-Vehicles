''' classifier.py

    Point classification using logistic regression

    Daniel Morris, Nov 2022

    python -m pip install -U scikit-learn
'''

import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import average_precision_score, precision_recall_curve
import matplotlib.pyplot as plt
from labeled_data import LabeledData

class Classifier:

    def __init__(self) -> None:
        self.cvec = np.array([])
        self.intercept = np.array([])
        self.logr = LogisticRegression(class_weight='balanced',solver='lbfgs')

    def fit(self, mydata ):
        self.logr.fit(mydata.data,mydata.labels)
        self.cvec = self.logr.coef_.ravel()
        self.intercept = np.array( self.logr.intercept_)

    def classify(self, mydata):
        ''' Returns scores '''
        if self.cvec.size==0:
            print('Must fit model before classifying data')
            raise Exception
        return (mydata.data * self.cvec[None,:]).sum(axis=1) + self.intercept[0]
    
    def average_precision(self, labels, scores):
        return average_precision_score(labels.astype(bool), scores)

    def plot_pr_curve(self, labels, scores, fignum='PR Curve', block=True, filesave=None):
        precision, recall, _ = precision_recall_curve(labels.astype(bool), scores)
        average_precision = self.average_precision(labels, scores)

        fig = plt.figure(fignum,figsize=(5,4))
        plt.plot(recall, precision, color='b', label='Precision-Recall curve')
        plt.xlabel('Recall')
        plt.ylabel('Precision')
        plt.title(f'Precision-Recall curve: AP={average_precision:.3f}')
        plt.legend(fontsize='small')
        plt.grid()

        if filesave:
            plt.savefig(filesave)
        plt.show(block=block)

        return average_precision

    def plot_line_on_axes(self, xchan=0, ychan=1, label='Line', color=[0,0,0], linestyle='-', block=True):
        ''' Plots 2D line on current axes corresponding to: 
            self.cvec[xchan]*x + self.cvec[ychan]*y + self.intercept = 0
        '''
        xminmax = np.array(plt.gca().get_xlim())
        yminmax = np.array(plt.gca().get_ylim())
        
        yvals = (-self.cvec[xchan]*xminmax - self.intercept[0]) / self.cvec[ychan]
        xvals = (-self.cvec[ychan]*yminmax - self.intercept[0]) / self.cvec[xchan]

        # find pair of locations that line intersects the axes:
        intersect = []
        if yvals[0]>=yminmax[0] and yvals[0]<=yminmax[1]:
            intersect.append( [xminmax[0],yvals[0]] )
        if yvals[1]>=yminmax[0] and yvals[1]<=yminmax[1]:
            intersect.append( [xminmax[1],yvals[1]] )
        if xvals[0]>=xminmax[0] and xvals[0]<=xminmax[1]:
            intersect.append( [xvals[0],yminmax[0]] )
        if xvals[1]>=xminmax[0] and xvals[1]<=xminmax[1]:
            intersect.append( [xvals[1],yminmax[1]] )
        
        if len(intersect):
            line = np.array(intersect)
            plt.plot( line[:,0], line[:,1], color=color, linestyle=linestyle, label=label )
        else:
            print('Line does not intersect axes')

        if block:
            plt.legend()
        plt.show(block=block)

    def plot_all_points(self, points, fignum='Points', title=None, block=True, filesave=None):

        target = points.get_subset(points.labels==1)
        clutter = points.get_subset(points.labels==0)
        Nplots = points.data.shape[1] // 2
        fig = plt.figure(fignum,figsize=(5*Nplots,4))
        for p in range(Nplots):
            plt.subplot(1,Nplots,p+1)
            xchan, ychan = 2*p, 2*p+1
            target.plot_points(xchan=xchan, ychan=ychan, c=[[.1,.8,.1]], label='Target', block=False)
            clutter.plot_points(xchan=xchan, ychan=ychan, c=[[.1,.1,.8]], label='Clutter', block=False)
            plt.legend(fontsize='small')
        plt.suptitle(title)
        if filesave:
            plt.savefig(filesave)
        plt.show(block=block)

    def plot_hist(self, labels, scores, fignum='Hist', block=True, filesave=None):

        average_precision = self.average_precision(labels, scores)
        negpos = [scores[labels==1], scores[labels==0]]

        fig = plt.figure(fignum,figsize=(5,4))
        plt.hist( negpos, bins=30, histtype='stepfilled', alpha=0.4, color=[[.1,.8,.1],[.1,.1,.8]], label=['Target','Clutter'] ) 
        plt.legend(fontsize='small')
        plt.xlabel('Scores')
        plt.ylabel('N per bin')
        plt.title(f'Distribution of Scores, AP={average_precision:.3f}')

        if filesave:
            plt.savefig(filesave)
        plt.show(block=block)

    def plot_results(self, points, scores, fignum='Result', block=True, filesave=None):
        ''' 
            points: data points
            scores: output of model.classify(points)
        '''
        if points.data.shape[1]>2:
            print(f'Not plotting hyperplane since data dimension of {points.data.shape[1]} is > 2')

        t_correct = points.get_subset( (points.labels==1) & (scores>0) ) 
        c_correct = points.get_subset( (points.labels==0) & (scores<=0) )
        all_wrong = LabeledData()
        all_wrong.append( points.get_subset( (points.labels==1) & (scores<=0) ) )
        all_wrong.append( points.get_subset( (points.labels==0) & (scores>0) ) )

        n_true_positives =  ((points.labels==1)  & (scores>0) ).sum()
        n_false_positives = ((points.labels==0)  & (scores>0) ).sum()
        n_false_negatives = ((points.labels==1)  & (scores<=0)).sum()

        precision = n_true_positives / (n_true_positives + n_false_positives)
        recall = n_true_positives / (n_true_positives + n_false_negatives)

        average_precision = self.average_precision(points.labels, scores)

        plt.figure(fignum,figsize=(5,4))
        xchan, ychan = 0, 1
        t_correct.plot_points(xchan=xchan, ychan=ychan, c=[[.1,.8,.1]], label='TP', block=False)
        c_correct.plot_points(xchan=xchan, ychan=ychan, c=[[.1,.1,.8]],  label='TN', block=False)
        all_wrong.plot_points(xchan=xchan, ychan=ychan, c='r',  label='FP+FN', block=False)
        if points.data.shape[1]==2:
            # only plot line if 2D data
            self.plot_line_on_axes(xchan=xchan, ychan=ychan, label='T=0', block=False)
        plt.legend(fontsize="small")
        plt.suptitle(f'AP: {average_precision:.3f}, Pr: {precision:.3f}, Recall: {recall:.3f}, Nchan: {points.data.shape[1]}')

        if filesave:
            plt.savefig(filesave)
        plt.show(block=block)



