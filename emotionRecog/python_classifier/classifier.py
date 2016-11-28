import numpy as np
from sklearn.svm import SVC
import matplotlib.pyplot as plt
from sklearn.svm import SVC
from sklearn.ensemble import RandomForestClassifier 
from sklearn.cross_validation import train_test_split
from sklearn.cross_validation import ShuffleSplit
from sklearn.grid_search import GridSearchCV
from sklearn.learning_curve import learning_curve


def plot_learning_curve(estimator, title, X, y, ylim=None, cv=None,
                        n_jobs=1, train_sizes=np.linspace(.1, 1.0, 5)):
    plt.figure()
    plt.title(title)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.xlabel("Training examples")
    plt.ylabel("Score")
    train_sizes, train_scores, test_scores = learning_curve(
        estimator, X, y, cv=cv, n_jobs=n_jobs, train_sizes=train_sizes)
    train_scores_mean = np.mean(train_scores, axis=1)
    train_scores_std = np.std(train_scores, axis=1)
    test_scores_mean = np.mean(test_scores, axis=1)
    test_scores_std = np.std(test_scores, axis=1)
    plt.grid()

    plt.fill_between(train_sizes, train_scores_mean - train_scores_std,
                     train_scores_mean + train_scores_std, alpha=0.1,
                     color="r")
    plt.fill_between(train_sizes, test_scores_mean - test_scores_std,
                     test_scores_mean + test_scores_std, alpha=0.1, color="g")
    plt.plot(train_sizes, train_scores_mean, 'o-', color="r",
             label="Training score")
    plt.plot(train_sizes, test_scores_mean, 'o-', color="g",
             label="Cross-validation score")

    plt.legend(loc="best")
    return plt


def SVM_classifier(X, y):
	X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=0)

	estimator = SVC(kernel='linear')

	cv = ShuffleSplit(X_train.shape[0], n_iter=10, test_size=0.2, random_state=0)

	gammas = np.logspace(-6, -1, 10)
	classifier = GridSearchCV(estimator=estimator, cv=cv, param_grid=dict(gamma=gammas))
	classifier.fit(X_train, y_train)

	title = 'Learning Curves (SVM, linear kernel, $\gamma=%.6f$)' %classifier.best_estimator_.gamma
	estimator = SVC(kernel='linear', gamma=classifier.best_estimator_.gamma)
	plot_learning_curve(estimator, title, X_train, y_train, cv=cv)
	plt.show()


	print(classifier.score(X_test, y_test))


def RF_classifier(X, y):
	X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=0)

	estimator = RandomForestClassifier()

	cv = ShuffleSplit(X_train.shape[0], n_iter=10, test_size=0.2, random_state=0)

	gammas = np.logspace(-6, -1, 10)
	param_grid = {"max_depth": [3, None],
              "max_features": [1, 3, 10],
              "min_samples_split": [1, 3, 10],
              "min_samples_leaf": [1, 3, 10],
              "bootstrap": [True, False],
              "criterion": ["gini", "entropy"]}
	classifier = GridSearchCV(estimator=estimator, cv=cv, param_grid=param_grid)
	classifier.fit(X_train, y_train)

	title = 'Learning Curves (Random Forest)'
	estimator = RandomForestClassifier(max_depth=classifier.best_estimator_.max_depth,
									   max_features=classifier.best_estimator_.max_features,
										min_samples_split=classifier.best_estimator_.min_samples_split,
										min_samples_leaf=classifier.best_estimator_.min_samples_leaf,
										bootstrap=classifier.best_estimator_.bootstrap,
										criterion=classifier.best_estimator_.criterion)
	plot_learning_curve(estimator, title, X_train, y_train, cv=cv)
	plt.show()

	print(classifier.score(X_test, y_test))



if __name__ == "__main__":
	X = []
	y = []
	for image in range(1, 94):
		histo = []
		file_name = "/home/theo/Documents/3D/Projet/emotionRecog/data/histograms/2_sad/" + str(image) + ".txt"
		with open(file_name, 'r') as f:
			line = f.read()
			line = line.split("(")
			line = line[1].split(")")[0]
			histo = [float(i) for i in line.split(", ")]
			X.append(histo)
			y.append(0)
		f.close()
		
	for image in range(1, 123):
		histo = []
		file_name = "/home/theo/Documents/3D/Projet/emotionRecog/data/histograms/2_joyful/" + str(image) + ".txt"
		with open(file_name, 'r') as f:
			line = f.read()
			line = line.split("(")
			line = line[1].split(")")[0]
			histo = [float(i) for i in line.split(", ")]
			X.append(histo)
			y.append(1)
		f.close()
	X = np.array(X)
	y = np.array(y)
	
	SVM_classifier(X, y)
	
	RF_classifier(X, y)
	


