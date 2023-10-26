import cv2
import argparse

#get every image in a folder and resize them

##### Argument parsing (parameters)
parser = argparse.ArgumentParser()
parser.add_argument('-image_name', help='file path and name of image', required=True) # filepath and image name
parser.add_argument('-height', required=True) # output height
parser.add_argument('-width', required=True)  # output width

args = parser.parse_args()

##### Image-resizing function
def resize_images(image_name, fheight, fwidth):
    #reads image
    image = (cv2.imread(image_name))
    height, width, _ = image.shape

    # sets output size
    dx = fwidth/width
    dy = fheight/height

    #resize image and return
    small_image = cv2.resize(image, None, fx = dx, fy = dy)
    cv2.imwrite(image_name, small_image)
    return small_image

##### main and function calls
def main():
    images = resize_images(args.image_name, int(args.height, base=10), int(args.width, base=10))

main()