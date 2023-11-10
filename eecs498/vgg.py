import torch
import torchvision
import torchvision.transforms as transforms
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import time

import math
from collections import OrderedDict

"""
Creating neural networks in pytorch is easy, because a lot of the nuts and bolts
involved in training are abstracted away from the user. All we need to do is
create a class for our neural network, in this case VGG. 

Layers are usually implemented as class attributes, and they are put together to
define the forward pass in the class's forward function. We also have to write
train and test functions, but you will not need to do this for this assignment.
However, I strongly encourage all students to take a look at these functions, as
well as the dataloading procedure in main().

To create layers, we can make use of nn.Sequential, which allows us to combine
multiple layers together in a single object. For example, if we want to
implement the following network

1. convolutional layer, input channels 3, output channels 8, filter size 3
2. max-pooling layer, size 2
3. ReLU
4. fully-connected layer (512->10),

we can do something like the following:

self.network = nn.Sequential(
                nn.Conv2d(3, 8, kernel_size=3, padding=1),
                nn.MaxPool2d(2),
                nn.ReLU(),
                nn.Linear(512,10)
                )

Note that the input size of the linear layer (512) will only be correct depending
on the input size of our images. In this case, the input size would need to be
16 x 16, since the max pooling layer (of size 2)  will decrease this size to 8x8
and there are 8 output channels, giving 8*8*8=512 features derived from the
input image. The sizes given in the comments below should work without you
having to figure any of this out.

Since we are working with CIFAR10, rather than imagenet, we have modified the
size of the various layers to work better for smaller images.

"""



class VGG(nn.Module):
    # You will implement a simple version of vgg11 (https://arxiv.org/pdf/1409.1556.pdf)
    # Since the shape of image in CIFAR10 is 32x32x3, much smaller than 224x224x3, 
    # the number of channels and hidden units are decreased compared to the architecture in paper
    def __init__(self):
        super(VGG, self).__init__()
        self.conv = nn.Sequential(
            # Stage 1
            # TODO: convolutional layer, input channels 3, output channels 8, filter size 3
            # TODO: max-pooling layer, size 2
            # We have commented out the function calls you need to make here, so
            # you can see an example for the rest of the network. Fill in the
            # ?s.

            #nn.Conv2d(?, ?, kernel_size=?, padding=1),
            #nn.MaxPool2d(?),
            nn.Conv2d(3, 8, kernel_size=3, padding=1),
            nn.MaxPool2d(2),
            
            # Stage 2
            # TODO: convolutional layer, input channels 8, output channels 16, filter size 3
            # TODO: max-pooling layer, size 2
            nn.Conv2d(8, 16, kernel_size=3, padding=1),
            nn.MaxPool2d(2),
            
            # Stage 3
            # TODO: convolutional layer, input channels 16, output channels 32, filter size 3
            # TODO: convolutional layer, input channels 32, output channels 32, filter size 3
            # TODO: max-pooling layer, size 2
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.Conv2d(32, 32, kernel_size=3, padding=1),
            nn.MaxPool2d(2),
            
            # Stage 4
            # TODO: convolutional layer, input channels 32, output channels 64, filter size 3
            # TODO: convolutional layer, input channels 64, output channels 64, filter size 3
            # TODO: max-pooling layer, size 2
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.Conv2d(64, 64, kernel_size=3, padding=1),
            nn.MaxPool2d(2),
            
            # Stage 5
            # TODO: convolutional layer, input channels 64, output channels 64, filter size 3
            # TODO: convolutional layer, input channels 64, output channels 64, filter size 3
            # TODO: max-pooling layer, size 2
            nn.Conv2d(64, 64, kernel_size=3, padding=1),
            nn.Conv2d(64, 64, kernel_size=3, padding=1),
            nn.MaxPool2d(2)
        )
        self.fc = nn.Sequential(
            # TODO: fully-connected layer (64->64)
            nn.Linear(64,64),
            
            # TODO: ReLU
            nn.ReLU(),
            
            # here you can try adding more fully-connected layers followed by
            # ReLU, if you want.
            
            # TODO: fully-connected layer (64->10)
            nn.Linear(64,10)

            # the softmax will be part of the cross entropy loss (defined
            # in main()) so we just need to have a linear layer with output size
            # equal to the number of classes (10). This is what is accomplished
            # by the layer you will implement above.

        )


    def forward(self, x):
        x = self.conv(x)
        # if you decide to change or add anything to conv(), you will need to
        # change x.view(-1, num_feats) where num_feats is the number of scalar
        # output features from conv(). You will then need to change the first
        # input layer in fc() to be num_feats as well.l 
        x = x.view(-1, 64)
        x = self.fc(x)
        return x


def train(trainloader, net, criterion, optimizer, device):
    for epoch in range(20):  # loop over the dataset multiple times
        start = time.time()
        running_loss = 0.0
        for i, (images, labels) in enumerate(trainloader):
            images = images.to(device)
            labels = labels.to(device)
            # zero the parameter gradients
            optimizer.zero_grad()
            # forward pass
            yhat = net.forward(images)
            loss = criterion(yhat, labels)
            # backward pass
            loss.backward()
            # optimize the network
            optimizer.step()
            # print statistics
            running_loss += loss.item()
            if i % 100 == 99:    # print every 2000 mini-batches
                end = time.time()
                print('[epoch %d, iter %5d] loss: %.3f eplased time %.3f' %
                      (epoch + 1, i + 1, running_loss / 100, end-start))
                start = time.time()
                running_loss = 0.0
    print('Finished Training')


def test(testloader, net, device):
    correct = 0
    total = 0
    with torch.no_grad():
        for data in testloader:
            images, labels = data
            images = images.to(device)
            labels = labels.to(device)
            outputs = net(images)
            _, predicted = torch.max(outputs.data, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
    print('Accuracy of the network on the 10000 test images: %d %%' % (
        100 * correct / total))


def main():
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    transform = transforms.Compose(
        [transforms.ToTensor(),
         transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])

    trainset = torchvision.datasets.CIFAR10(root='./data', train=True,
                                        download=True, transform=transform)
    trainloader = torch.utils.data.DataLoader(trainset, batch_size=100,
                                          shuffle=True)

    testset = torchvision.datasets.CIFAR10(root='./data', train=False,
                                       download=True, transform=transform)
    testloader = torch.utils.data.DataLoader(testset, batch_size=100,
                                         shuffle=False)
    net = VGG().to(device)
    criterion = nn.CrossEntropyLoss()
    
    optimizer = optim.Adam(net.parameters(), lr=0.001)
    #optimizer = optim.SGD(net.parameters(), lr=0.001, momentum=0.9)

    train(trainloader, net, criterion, optimizer, device)
    test(testloader, net, device)
    

if __name__== "__main__":
    main()
   
