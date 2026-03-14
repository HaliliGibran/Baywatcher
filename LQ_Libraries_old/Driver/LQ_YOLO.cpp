#include "LQ_YOLO.hpp"

using namespace std;
using namespace cv::dnn;
using namespace std::chrono;
using namespace std::this_thread;
using cv::Mat;
using std::cout;
using std::endl;
using std::string;
using std::vector;

#define Category_NUM        ( 5 )       // 分类数量
#define MAX_RELIABLE_NUM    ( 100 )     // 最大高置信度数据数量
#define DATA_LEN            ( 10 )      // 数据长度 Category_NUM + 5
#define img_size            ( 64 )      // 输入图片尺寸

// 分类结果存储
static const vector<string> class_name = { // 分类名称
    "Mouse",
    "Printer",
    "drill", 
    "Wrench",
    "Screwdriver"
};

// NMS处理后的边界框存储结构
struct BoundingBox
{
    float  x1, y1, x2, y2;  // 边界框坐标
    float  confidence;      // 置信度
    int    category;        // 类别索引
    string category_name;   // 类别名称
};

// 存储最佳检测结果的全局变量
struct BestDetection
{
    BoundingBox box;
    bool        detected;
    // 结构体默认构造函数
    BestDetection() : detected(false) {}
};

/******************* 获取高置信度数据并分类 ********************/

void Obtain_high_confidence(const Mat &result,
                            vector<vector<float>> &high_confidence_data,
                            vector<vector<float>> classified_data[],
                            int &high_confidence_count,
                            int category_count[],
                            float conf = 0.75, int len_data = 10)
{
    float *pdata = (float *)result.data;

    // 重置计数器和分类数据
    high_confidence_count = 0;
    for (int i = 0; i < Category_NUM; i++)
    {
        classified_data[i].clear();
        category_count[i] = 0;
    }

    for (int i = 0; i < result.total() / len_data; i++)
    {
        if (pdata[4] > conf)
        {
            // 检查是否超过数组最大容量
            if (high_confidence_count >= MAX_RELIABLE_NUM)
            {
                cout << "警告：高置信度数据数量超过最大容量 " << MAX_RELIABLE_NUM << endl;
                break;
            }

            // 复制高置信度数据到数组
            vector<float> current_data;
            for (int j = 0; j < len_data; j++)
            {
                current_data.push_back(pdata[j]);
            }
            high_confidence_data.push_back(current_data);

            // 分类处理：找到最大相似度的类别
            int max_category = 0;
            float max_similarity = pdata[len_data - Category_NUM]; // 第一个相似度值

            for (int k = 1; k < Category_NUM; k++)
            {
                float similarity = pdata[len_data - Category_NUM + k];
                if (similarity > max_similarity)
                {
                    max_similarity = similarity;
                    max_category = k;
                }
            }

            // 将数据添加到对应类别
            classified_data[max_category].push_back(current_data);
            category_count[max_category]++;

            // 增加计数器
            high_confidence_count++;
        }
        pdata += len_data;
    }
}

/******************* 计算两个矩形框的IoU ********************/

float CalculateIoU(const vector<float> &box1, const vector<float> &box2)
{
    // 假设数据格式: [x_center, y_center, width, height, confidence, ...]
    float x1 = box1[0] - box1[2] / 2;
    float y1 = box1[1] - box1[3] / 2;
    float x2 = box1[0] + box1[2] / 2;
    float y2 = box1[1] + box1[3] / 2;

    float x3 = box2[0] - box2[2] / 2;
    float y3 = box2[1] - box2[3] / 2;
    float x4 = box2[0] + box2[2] / 2;
    float y4 = box2[1] + box2[3] / 2;

    // 计算交集区域
    float inter_x1 = max(x1, x3);
    float inter_y1 = max(y1, y3);
    float inter_x2 = min(x2, x4);
    float inter_y2 = min(y2, y4);

    float inter_area = max(0.0f, inter_x2 - inter_x1) * max(0.0f, inter_y2 - inter_y1);

    // 计算并集区域
    float area1 = box1[2] * box1[3];
    float area2 = box2[2] * box2[3];
    float union_area = area1 + area2 - inter_area;

    if (union_area == 0)
        return 0.0f;
    return inter_area / union_area;
}

/******************* 计算两个中心点的距离 ********************/

float CalculateCenterDistance(const vector<float> &box1, const vector<float> &box2)
{
    float dx = box1[0] - box2[0];
    float dy = box1[1] - box2[1];
    return sqrt(dx * dx + dy * dy);
}

/******************* 非极大值抑制处理单个类别 ********************/

vector<vector<float>> ApplyNMS_per_category(vector<vector<float>> &category_boxes,
                                            float iou_threshold      = 0.5,
                                            float distance_threshold = 0.2)
{
    // 检查输入是否为空
    if (category_boxes.empty())
    {
        return {};
    }

    // 按置信度降序排序，优先处理高置信度框
    sort(category_boxes.begin(), category_boxes.end(),
        [](const vector<float> &a, const vector<float> &b) { return a[4] > b[4]; }); // 按置信度降序
    
    // 存储抑制后的框和抑制标记
    vector<vector<float>> suppressed_boxes;
    vector<bool> is_suppressed(category_boxes.size(), false);

    // 遍历所有框，执行非极大值抑制
    for (std::size_t i = 0; i < category_boxes.size(); i++)
    {
        // 跳过已抑制的框
        if (is_suppressed[i])
            continue;

        // 将当前框加入结果集
        suppressed_boxes.push_back(category_boxes[i]);

        // 与后续框比较，抑制重叠严重的框
        for (std::size_t j = i + 1; j < category_boxes.size(); j++)
        {
            // 跳过已抑制的框
            if (is_suppressed[j])
                continue;

            // 计算IoU和中心点距离
            float iou = CalculateIoU(category_boxes[i], category_boxes[j]);
            float center_distance = CalculateCenterDistance(category_boxes[i], category_boxes[j]);

            // 如果IoU超过阈值或中心点距离过近，则抑制该框
            if (iou > iou_threshold || center_distance < distance_threshold)
            {
                is_suppressed[j] = true;
            }
        }
    }
    return suppressed_boxes;
}

/******************* 找到最佳检测结果（置信度最高的物品） ********************/

BestDetection FindBestDetection(vector<vector<float>> classified_data[])
{
    BestDetection best;
    float highest_confidence = 0.0f;

    for (int i = 0; i < Category_NUM; i++)
    {
        if (!classified_data[i].empty())
        {
            // 对每个类别应用NMS
            vector<vector<float>> nms_boxes = ApplyNMS_per_category(classified_data[i]);

            // 在每个类别中找置信度最高的框（NMS后）
            for (const auto &box_data : nms_boxes)
            {
                if (box_data[4] > highest_confidence)
                {
                    highest_confidence = box_data[4];

                    // 填充最佳检测结果
                    best.box.x1 = box_data[0] - box_data[2] / 2;
                    best.box.y1 = box_data[1] - box_data[3] / 2;
                    best.box.x2 = box_data[0] + box_data[2] / 2;
                    best.box.y2 = box_data[1] + box_data[3] / 2;
                    best.box.confidence = box_data[4];
                    best.box.category = i;
                    best.box.category_name = class_name[i];
                    best.detected = true;
                }
            }
        }
    }
    return best;
}

/******************* 初始化所有数据结构 ********************/

void InitializeDataStructures(vector<vector<float>> &high_confidence_data,
                              vector<vector<float>> classified_data[],
                              int &high_confidence_count,
                              int category_count[])
{
    // 清空所有数据
    high_confidence_data.clear();
    high_confidence_count = 0;

    for (int i = 0; i < Category_NUM; i++)
    {
        classified_data[i].clear();
        category_count[i] = 0;
    }
}

/* OpenCV 中的 DNN(深度神经网络)模块, 用于 YOLO 目标检测, 完整命名为 cv::dnn::Net */
Net net;
// 定义每帧需要使用的数据结构
vector<vector<float>> high_confidence_data;
vector<vector<float>> classified_data[Category_NUM];
int high_confidence_count = 0;
int category_count[Category_NUM] = {0};
// 存储最佳检测结果的变量
BestDetection current_best_detection;

/********************************************************************************
 * @brief   初始化YOLO目标检测模型.
 * @param   none.
 * @return  none.
 * @note    加载预训练的ONNX格式的YOLO模型文件，为后续的目标检测做准备。
 *          该函数会读取 "best.onnx" 文件并初始化神经网络。
 ********************************************************************************/
void YOLO_Init()
{
    // 加载模型和配置文件
    net = readNetFromONNX("best.onnx");
}

void YOLO_Detecion(const Mat &image)
{
    // 初始化数据结构（每帧清空）
    InitializeDataStructures(high_confidence_data, classified_data,
                             high_confidence_count, category_count);

    // 预处理图像
    Mat blob = dnn::blobFromImage(image, 1.0 / 255.0, Size(img_size, img_size), Scalar(), true);

    // 网络推理
    net.setInput(blob);
    vector<Mat> netoutput;
    vector<string> out_name = {"output0"};
    net.forward(netoutput, out_name);
    Mat result = netoutput[0];

    // 处理结果
    Obtain_high_confidence(result, high_confidence_data, classified_data,
                           high_confidence_count, category_count);

    // 找到最佳检测结果（置信度最高的物品）
    current_best_detection = FindBestDetection(classified_data);

    cout << " 物体: " << current_best_detection.box.category_name
         << " 置信度: " << current_best_detection.box.confidence << endl;
}