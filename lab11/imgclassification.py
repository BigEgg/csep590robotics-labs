#!/usr/bin/env python

##############
# Your name:
##############

import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color


class ImageClassifier:

    def __init__(self):
        self.classifer = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)

        # create one large array of image data
        data = io.concatenate_images(ic)

        # extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]

        return (data, labels)

    def extract_image_features(self, data):
        # Please do not modify the header above

        # extract feature vector from image data

        feature_data = []
        for original_img in data:
            img = color.rgb2gray(original_img)

            blurred_img = filters.gaussian(img, sigma = 3)
            # io.imsave(f'./hog/{i}_blurred.bmp', blurred_img)

            (hog_features, hog_image) = feature.hog(img, block_norm="L2", orientations=9, pixels_per_cell=(25, 25),
                    cells_per_block=(4, 4), visualise=True)
            # hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 10))
            # io.imsave(f'./hog/{i}.bmp', hog_image_rescaled)
            feature_data.append(hog_features)

        # Please do not modify the return type below
        return (feature_data)

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above

        # train model and save the trained model to self.classifier

        self.classifer = svm.LinearSVC().fit(train_data, train_labels)


    def predict_labels(self, data):
        # Please do not modify the header

        # predict labels of test data using trained model in self.classifier
        # the code below expects output to be stored in predicted_labels

        predicted_labels = self.classifer.predict(data)

        # Please do not modify the return type below
        return predicted_labels


def main():

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')

    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    test_data = img_clf.extract_image_features(test_raw)

    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)
    print("Training success")

    predicted_labels = img_clf.predict_labels(train_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n", metrics.confusion_matrix(train_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))

    # test model
    predicted_labels = img_clf.predict_labels(test_data)
    print("\nTest results")
    print("=============================")
    print("Confusion Matrix:\n", metrics.confusion_matrix(test_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))


if __name__ == "__main__":
    main()
