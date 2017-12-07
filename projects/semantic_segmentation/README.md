# UDACITY SELF-DRIVING CAR ENGINEER NANODEGREE

# Semantic Segmentation Project (Advanced Deep Learning)

## Introduction

The goal of this project is to construct a fully convolutional neural network based
on the VGG-16 image classifier architecture for performing semantic segmentation to identify drivable
road area from an car dashcam image (trained and tested on the KITTI data set).

## Approach

### Architecture

A pre-trained VGG-16 network was converted to a fully convolutional network by converting
the final fully connected layer to a 1x1 convolution and setting the depth equal to the number
of desired classes

Performance is improved through the use of skip connections, performing 1x1 convolutions on previous VGG layers
(in this case, layers 3 and 4) and
adding them element-wise to upsampled (through transposed convolution)
lower-level layers (i.e. the 1x1-convolved layer 7 is
upsampled before being added to the 1x1-convolved layer 4).

Each convolution and transpose convolution layer includes a kernel initializer and regularizer

### Optimizer

The loss function for the network is cross-entropy, and an Adam optimizer is used.


## Files

    vizualize_images.ipynb
        vizualize the input images

    main.py
        main program that trains the FCN

    helper.py
        helper functions

    tests.py
        test/validation programs

### Training

The hyperparameters used for training are:

  - keep_prob: 0.5
  - learning_rate: 0.00095
  - epochs: 30
  - batch_size: 5

## Results

Loss per batch tends to average below 0.200 after two epochs and below 0.100 after ten epochs.
Average loss per batch at epoch 10: 20: 0.054, at epoch 30: 0.072

The results are created in output directory.

### Samples

output/samples director contains few sample images from the output of the fully convolutional network,
with the segmentation class overlaid upon the original image in green.


References:

1. [KITTI](http://www.cvlibs.net/datasets/kitti/) dataset
2. [Cityscapes](https://www.cityscapes-dataset.com/) dataset
3. [FCN](https://people.eecs.berkeley.edu/~jonlong/long_shelhamer_fcn.pdf) ANN

---

Run the following command to run the project:
```
python main.py
```

More details can be found in Semantic_Segmentation_Project_Reflection.pdf 
