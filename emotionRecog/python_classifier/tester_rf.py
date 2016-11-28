from sklearn.externals import joblib
import sys
import numpy as np
model = joblib.load('rf_model.pkl')

line = sys.argv[1]
line = line.split("(")
line = line[1].split(")")[0]
histo = [float(i) for i in line.split(", ")]
histo = np.array(histo)

print(model.predict(histo.reshape(1, -1))[0])
