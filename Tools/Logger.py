import os
import csv

class Logger(object):
    def __init__(self, folderName):
        if os.name == 'posix':
            self.dir = os.getcwd()+'/log/'+folderName+'/'
        else:
            None
            # self.dir = os.getcwd()+'\log\+folderName+'\'

    def WriteToLog(self, data, fileName):
        if type(data) == "pandas.core.frame.DataFrame":
            data.to_csv(self.dir+fileName+'.csv', index=False, sep=',')
        else:
            None
            # csvFile = open('')
