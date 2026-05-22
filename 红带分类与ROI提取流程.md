输入图像

对每个像素定义：

red_score = 2R - G - B
dom = R - max(G, B)

其中：
- `R`、`G`、`B` 为 BGR 图像中的三个通道值
- 实现时应使用有符号整型或浮点型计算，不允许用 `uint8` 直接做减法

严格红带掩码固定为：

red_score >= 140
R >= 90
dom >= 80

该阈值用于：
- 搜索框内红带提取
- 搜索框 x 包络中的红带部分

搜索带固定为：

y = 160..320

更准确地说：
- `search_y_min = 160`
- `search_y_max = 320`

这里采用上闭下开或闭区间的实现细节由代码决定，但几何意义固定为“图像中部到下中部这一条检测带”。

白带参考行固定为：

white_reference_row_y = 320

用于在该行上检测白带 x 范围。

初始基础搜索框为：

x = 全宽
y = 160..320

在 `y = 320` 参考行上检测白带范围，得到：

white_x_range = [white_x_min, white_x_max]

当前白带判定口径是：
- 低饱和
- 高亮
- 在参考行上取最长或合并后的有效白带范围

在基础搜索框内，用严格红带阈值：

red_score >= 140, R >= 90, dom >= 80

生成基础红色掩码，再取其最小 x 包络：

red_x_range = [red_x_min, red_x_max]

最终搜索框的 x 由：

白带 x 范围 与 严格红带 x 范围 的最大包络

即：
x0 = min(white_x_min, red_x_min)
x1 = max(white_x_max, red_x_max)

如果某一项不存在，则使用另一项；如果两项都不存在，则退回全宽。

最终搜索框定义为：

x = 白带与红带的最大包络
y = 160..320

并最终裁剪到图像范围内。

只在最终搜索框内生成严格红带掩码：

red_score >= 140
R >= 90
dom >= 80

红带掩码阶段固定：

不开运算
不闭运算
不做 open+close
不做 blur

即：
no morphology

如果当前红色掩码在搜索框内**连通到搜索框边界**，则说明搜索框可能裁断红带，需要外推。

边界包括：
- 左边界
- 上边界
- 右边界
- 下边界

固定外推步长：
expand_step = 8 px

固定最大外推次数：

max_expand_steps = 12

每一轮：
1. 检查红色掩码是否贴左 / 上 / 右 / 下边界
2. 哪一边贴边，就只向那一边外扩 `8 px`
3. 重新生成搜索框
4. 重新在新搜索框内生成严格红带掩码
5. 重复直到：
   - 红色掩码不再贴边
   - 或达到 `12` 轮



对最终红色掩码做：
connectedComponentsWithStats

只保留满足以下最小阈值的红色连通域：

area >= 80
width >= 12
height >= 3

在所有满足最小阈值的连通域中：

取最靠下的红块作为红带

若有多个连通域底边相同，则优先面积更大的。

对最终红带输出：
- `candidate_area`
- `candidate_center_x`
- `candidate_center_y`
- `candidate_width`
- `candidate_height`
- `blob_box = [x, y, w, h]`

其中：
candidate_center_x = x + w / 2
candidate_center_y = y + h / 2


对每一张成功提取出红带的图，记：
- `x = 红带候选框中心 x`
- `y = 红带候选框中心 y`
- `A = 红带候选面积 candidate_area`

统一拟合形式写成：
A = a*x + b*y + c


标识块回归函数
f_marker(x,y) = -0.178636*x + 13.850833*y - 1926.156613

路障回归函数

f_roadblock(x,y) = -4.485307*x + 90.209323*y - 9060.983073

阈值函数固定为两者平均：

f_threshold(x,y) = 0.5 * (f_marker(x,y) + f_roadblock(x,y))

化简后固定为：
f_threshold(x,y) = -2.331971*x + 52.030078*y - 5493.569843


- 若 `candidate_area < f_threshold(x,y)`，判为 `marker`
- 若 `candidate_area > f_threshold(x,y)`，判为 `roadblock`

这里默认不单独定义模糊带。



只有判定为 `marker` 的红带，才继续做 ROI。

输入为：
- 红带 contour

对红带 contour 做：

minAreaRect

得到红带最小旋转矩形四点：

blob_quad

在该最小旋转矩形中：
1. 找两条长边
2. 取其中**靠上的那一条**
3. 记其端点为：

top_edge_raw = [p_left, p_right]

将 `blob_quad` 映射到 `IPM/final` 坐标系。

在 `IPM/final` 中：
1. 取“靠上的长边”作为正方形的下边
2. 边长取该长边长度
3. 沿其上法线方向向上构造一个正方形

得到：

roi_quad_final

将该 `IPM` 正方形反投影回原图，得到：

roi_quad

对 `roi_quad` 做透视变换，输出最终识别 ROI 图像。

默认输出大小：
64 x 64

若目标被判为 `roadblock`：
- 不生成 ROI
- 不进入后续识别模型
- 直接输出路障结果





最终执行摘要
1. 在 `y=160..320` 内建立基础搜索框  
2. 用白带与严格红带最大包络确定最终搜索框 `x`  
3. 在搜索框内按 `red_score>=140, R>=90, dom>=80` 生成严格红带掩码  
4. 不做形态学  
5. 若红色贴边，则按 `8 px` 步长逐层外推，最多 `12` 次  
6. 对红色连通域做连通域分析  
7. 取最靠下且满足 `area>=80, width>=12, height>=3` 的红块作为红带  
8. 计算红带中心 `(x, y)` 和面积 `A`  
9. 若 `A < f_threshold(x,y)`，判为 `marker`  
10. 若 `A > f_threshold(x,y)`，判为 `roadblock`  
11. `roadblock` 直接结束，不生成 ROI  
12. `marker` 对红带 contour 做 `minAreaRect`  
13. 取旋转矩形靠上的长边  
14. 在 IPM 中以上长边为下边，向上构造正方形 ROI  
15. 反投影回原图并透视变换，得到最终识别 ROI
