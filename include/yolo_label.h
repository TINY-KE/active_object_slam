//
// Created by zhjd on 9/28/22.
//

#ifndef ROS_EVO_YOLO_LABEL_H
#define ROS_EVO_YOLO_LABEL_H

static const char *yolo_id[] = {
        "person人类",  //0
        "bicycle自行车", "car汽车", "motorcycle", "airplane", "bus",   //1
        "train", "truck", "boat", "traffic light",   "fire hydrant", //6
        "stop sign停止标", "parking meter", "bench", "bird", "cat", //11
        "dog", "horse", "sheep", "cow",  "elephant", //16
        "bear", "zebra", "giraffe", "backpack背包", "umbrella雨伞", //21
        "handbag手提包", "tie领带", "suitcase手提箱", "frisbee",  "skis", //26
        "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", //31
        "skateboard", "surfboard",  "tennis racket", "bottle瓶子", "wine glass酒杯", //36
        "cup杯子", "fork", "knife", "spoon", "bowl碗", //41
        "banana香蕉", "apple苹果",   "sandwich三明治", "orange橙子", "broccoli", //46
        "carrot", "hot dog热狗", "pizza", "donut", "cake蛋糕", //51
        "chair椅子", "couch沙发",  "potted plant盆栽", "bed床", "dining table餐桌",//56
        "toilet", "tv电视", "laptop笔记本电脑", "mouse鼠标", "remote遥控器", //61
        "keyboard键盘", "cell phone手机",  "microwave微波炉", "oven烤箱", "toaster烤面包机", //66
        "sink水槽", "refrigerator冰箱", "book书", "clock钟", "vase花瓶", //71
        "scissors", "teddy bear泰迪熊",  "hair drier", "toothbrush"};//76

//std::vector<std::string> yolo_id = {
//    "person人类",  //0
//    "bicycle自行车", "car汽车", "motorcycle", "airplane", "bus",   //1
//    "train", "truck", "boat", "traffic light",   "fire hydrant", //6
//    "stop sign停止标", "parking meter", "bench", "bird", "cat", //11
//    "dog", "horse", "sheep", "cow",  "elephant", //16
//    "bear", "zebra", "giraffe", "backpack背包", "umbrella雨伞", //21
//    "handbag手提包", "tie领带", "suitcase手提箱", "frisbee",  "skis", //26
//    "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", //31
//    "skateboard", "surfboard",  "tennis racket", "bottle瓶子", "wine glass酒杯", //36
//    "cup杯子", "fork", "knife", "spoon", "bowl碗", //41
//    "banana香蕉", "apple苹果",   "sandwich三明治", "orange橙子", "broccoli", //46
//    "carrot", "hot dog热狗", "pizza", "donut", "cake蛋糕", //51
//    "chair椅子", "couch沙发",  "potted plant盆栽", "bed床", "dining table餐桌",//56
//    "toilet", "tv电视", "laptop笔记本电脑", "mouse鼠标", "remote遥控器", //61
//    "keyboard键盘", "cell phone手机",  "microwave微波炉", "oven烤箱", "toaster烤面包机", //66
//    "sink水槽", "refrigerator冰箱", "book书", "clock钟", "vase花瓶", //71
//    "scissors", "teddy bear泰迪熊",  "hair drier", "toothbrush"};//76

#endif //ROS_EVO_YOLO_LABEL_H
