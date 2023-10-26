#!/bin/bash

# Script Name: yoloscript
# Author: Ryan Sundermeyer, Joe Loffrees
# Date: 10/22/2023
# Description: Automates the terminal portion of training an AI model in yolov5

FOLDER=("$1")   # the name of the folder (string)
EPOCHS=("$2")   # number of epochs (integer)
WEIGHT=("$3")   # weight (string) -usually give it an empty string
shift 3         # Shift the argument list to remove the first three arguments.
CLASSES=("$@")  # class array (strings)
WORKSPACE="$(pwd)/vision/models"
IMG_HEIGHT=640
IMG_WIDTH=640


##### print parameters and help
echo "Read 'yolohelp.txt' for help running this program"
echo "FOLDER: $FOLDER"
echo "EPOCHS: $EPOCHS"
echo "WEIGHT: $WEIGHT"
echo "WORKSPACE: $WORKSPACE"

# Print the entire array
echo "OBJECTS: ${CLASSES[*]}"

##### create workspace 
mkdir vision/models/"$FOLDER" || return
cd    vision/models/"$FOLDER" || return
mkdir "yolo_labels"
cd    "yolo_labels"           || return
mv ~/Downloads/"$FOLDER".zip .
unzip -q       "$FOLDER".zip
rm             "$FOLDER".zip
cd ..

echo "Cloning yolov5..."
git clone https://github.com/ultralytics/yolov5 >> output.log
echo Installing requirements...
pip install -r ../standard/requirements.txt     >> output.log

##### roboflow at home
mkdir -p yolov5/train/images
mkdir    yolov5/train/labels
mkdir -p yolov5/test/images
mkdir    yolov5/test/labels
mkdir -p yolov5/valid/images
mkdir    yolov5/valid/labels

# copies training files to specific folders and resizes them
moveto() {
    local c="$1"
    local n="$2"
    local type="$3"
    local offset=$(((c + n) % 10))

    if [ $offset -eq 0 ]; then # if offset is 0
        cp      "$WORKSPACE/../datasets/$FOLDER/$c.jpg"            "$WORKSPACE/$FOLDER/yolov5/$type/images"
        python3 "$WORKSPACE/standard/image_resizer.py" -image_name "$WORKSPACE/$FOLDER/yolov5/$type/images/$c.jpg" -height $IMG_HEIGHT -width $IMG_WIDTH
        cp      "$WORKSPACE/$FOLDER/yolo_labels/$c.txt"            "$WORKSPACE/$FOLDER/yolov5/$type/labels"
    fi
}

echo "Resizing images..."

# for loop moves files to folders for use by yolov5: 70% train, 20% test, 10% valid
count=1
for file in "../../datasets/$FOLDER"/*; do
    moveto "$count" 0 "train"
    moveto "$count" 1 "train"
    moveto "$count" 2 "train"
    moveto "$count" 3 "test"
    moveto "$count" 4 "train"
    moveto "$count" 5 "train"
    moveto "$count" 6 "train"
    moveto "$count" 7 "test"
    moveto "$count" 8 "train"
    moveto "$count" 9 "valid"

    ((count++))
done

touch yolov5/data/data.yaml

CLASS_STR=""    # string for each class/object
# for loop creates an array in the correct format including strings for all the classes to put in data.yaml
ISFIRST="true"
for class in "${CLASSES[@]}"; do
    if [ "$ISFIRST" = "true" ]; then
        CLASS_STR="$class"
        ISFIRST="false"
    else
        CLASS_STR="$CLASS_STR, $class"
    fi
done

# emulating data.yaml - DO NOT EDIT
yaml_content=$(cat <<EOL
train: ../train/images
val: ../valid/images
test: ../test/images

nc: ${#CLASSES[@]}
names: [$CLASS_STR]
EOL
)

# Create the data.yaml file with the specified content
echo "$yaml_content"          > yolov5/data/data.yaml
echo "nc: ${#CLASSES[@]}"     > yolov5/models/yolov5m.yaml
cat ../standard/yolov5m.yaml >> yolov5/models/yolov5m.yaml

# run yolov5 AI model
cd yolov5 || return
echo "Running model..."
python3 "train.py" --weights "$WEIGHTS" --imgsz "640" --cfg "models/yolov5m.yaml" --epochs "$EPOCHS" --data "data/data.yaml"
mv runs/train/exp/weights/best.pt ../model.pt