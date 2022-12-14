# Training stop detection model.

## Xgboost model

- **data input**

  - orientation diff between previous record and current.
  ```
  Vector3d pre_ornt = ornt_data.row(i-1);
  Vector3d current_ornt = ornt_data.row(i);
  Vector3d ornt_diff = current_ornt - pre_ornt;
  ```

  - normalized accelerometer in b frame and n frame.
  ```
  Vector4d q = quaternions.GetQFromEuler(current_ornt);
  Matrix3d dcm = quaternions.GetDCMFromQ(q);

  Vector3d acc_v = acc_data;
  Vector3d acc_n = dcm * acc_v;
  Vector3d acc_v_norm = Normalise(acc_v);
  Vector3d acc_n_norm = Normalise(acc_n);
  ```

  - normalized gravity in b frame and n frame.
  ```
  Vector3d g_v = g_data;
  Vector3d g_n = dcm * g_v;
  Vector3d g_v_norm = Normalise(g_v);
  Vector3d g_n_norm = Normalise(g_n);
  ```

  - diff between accelerometer and gravity in n frame.
  ```
  Vector3d a_diff = acc_n - g_n;
  ```

  - normalized gyroscope.
  ```
  Vector3d gyro_v = gyro_data;
  Vector3d gyro_v_norm = Normalise(gyro_v);
  ```

  - normalized magnetic diff between previous record and current in n frame.
  ```
  Vector3d pre_mag_v = mag_data.row(i-1);
  Vector3d current_mag_v = mag_data.row(i);
  Vector3d mag_v_diff = (dcm * current_mag_v) - (dcm * pre_mag_v);
  Vector3d mag_v_norm = accelerometer.Normalise(mag_v_diff);
  ```

  - examples:

| acc_b_norm_x | acc_b_norm_y | acc_b_norm_z | acc_n_norm_x | acc_n_norm_y | acc_n_norm_z | g_b_norm_x   | g_b_norm_y   | g_b_norm_z  | g_n_norm_x   | g_n_norm_y  | g_n_norm_z  | gyro_norm_x  | gyro_norm_y  | gyro_norm_z  | mag_n_norm_x | mag_n_norm_y | mag_n_norm_z | mag_n_diff_norm_x | mag_n_diff_norm_y | mag_n_diff_norm_z | acc_g_n_diff_x | acc_g_n_diff_y | acc_g_n_diff_z | ornt_diff_x | ornt_diff_y | ornt_diff_z | label |
| ------------ | ------------ | ------------ | ------------ | ------------ | ------------ | ------------ | ------------ | ----------- | ------------ | ----------- | ----------- | ------------ | ------------ | ------------ | ------------ | ------------ | ------------ | ----------------- | ----------------- | ----------------- | -------------- | -------------- | -------------- | ----------- | ----------- | ----------- | ----- |
| 0.129066189  | 0.489784426  | 0.862237285  | -0.504075138 | 0.568416979  | 0.65023872   | 0.161449964  | 0.394004726  | 0.904817211 | -0.406554964 | 0.597145075 | 0.691470043 | 0.457245866  | -0.07097188  | -0.886503925 | 0.482868564  | 0.285974018  | 0.827681588  | 3.382281287       | 2.003121847       | 5.797544418       | -1.374394617   | 0.189218618    | 0.134366798    | -2.456686   | 2.903637    | -2.6312     | 1     |
| 0.098092756  | 0.441567958  | 0.891849511  | -0.417273415 | 0.538611451  | 0.731970356  | 0.115516734  | 0.376209022  | 0.919305529 | -0.350114553 | 0.550039758 | 0.758205819 | -0.52091589  | 0.693465674  | -0.497746918 | 0.485898133  | -0.436930527 | -0.756964146 | 1.580799524       | -1.421490477      | -2.462673718      | -0.715750162   | -0.038637689   | -0.157519063   | 2.954984    | -1.104863   | -1.016671   | 1     |
| 0.128268463  | 0.385579444  | 0.913715324  | -0.340256272 | 0.575695299  | 0.74350561   | 0.11413304   | 0.384831439  | 0.915903058 | -0.34341474  | 0.563926869 | 0.751034489 | 0.785250568  | -0.150309687 | -0.60065676  | 0.722509897  | 0.322986569  | 0.611276635  | 0.872183616       | 0.389895827       | 0.737907491       | 0.210543669    | -0.188356055   | -0.466219017   | 0.05889     | 0.534217    | -1.278368   | 1     |
| 0.090377072  | 0.506492751  | 0.857494652  | -0.430280858 | 0.540274825  | 0.723160768  | 0.0714945    | 0.373371378  | 0.924922889 | -0.305098583 | 0.513380471 | 0.80209435  | -0.40415878  | 0.907416845  | -0.115110168 | -0.018786556 | -0.478580999 | -0.877842408 | -0.056784486      | -1.446565096      | -2.653377777      | -1.56801271    | 0.690716987    | -0.202952674   | 2.683109    | -0.709583   | 0.002222    | 1     |
| 0.003769136  | 0.413187081  | 0.910638364  | -0.273759419 | 0.441200565  | 0.854633162  | -0.006953619 | 0.349818259  | 0.936791777 | -0.214272881 | 0.418783958 | 0.882443839 | -0.299333594 | 0.767442181  | -0.566949644 | 0.452037207  | -0.074923127 | -0.888846943 | 1.474141036       | -0.244332222      | -2.898623682      | -0.583401164   | 0.219646233    | -0.273327666   | 4.845336    | -1.44749    | -2.451917   | 1     |
| -0.012789521 | 0.414001052  | 0.910186551  | -0.238513744 | 0.437943501  | 0.866785258  | -0.033002042 | 0.360327767  | 0.932241796 | -0.192046192 | 0.40733654  | 0.892857885 | 0.761586392  | 0.040498834  | -0.646796732 | 0.696550054  | -0.020242884 | -0.717222593 | 1.845812816       | -0.053642339      | -1.900593713      | -0.3771996     | 0.155847649    | -0.54158446    | 1.602177    | 0.644123    | -1.650386   | 1     |
| -0.08225435  | -0.306475501 | 0.948317979  | -0.278243257 | 0.417762745  | 0.864901716  | -0.059733105 | -0.286562656 | 0.956197574 | -0.263618426 | 0.394924855 | 0.880079363 | -0.425352445 | -0.817550551 | -0.388183453 | 0.75555205   | -0.627310411 | -0.188739898 | 0.464302059       | -0.385494971      | -0.115984495      | -0.155815903   | 0.242576808    | -0.110514225   | -0.023439   | 0.017182    | -0.004636   | 0     |
| -0.072043    | -0.295740867 | 0.952547713  | -0.269786878 | 0.407713857  | 0.872344227  | -0.05961394  | -0.287154194 | 0.956027535 | -0.264061745 | 0.395357894 | 0.879751971 | -0.463747259 | -0.805428147 | -0.369085328 | -0.402699867 | 0.40874646   | -0.81899887  | -0.229401034      | 0.232845521       | -0.466548918      | -0.066589347   | 0.136967852    | -0.038958311   | -0.006482   | -0.035383   | 0.009744    | 0     |
| -0.06279196  | -0.294665    | 0.953535373  | -0.270584795 | 0.399787014  | 0.87575922   | -0.059551967 | -0.287931685 | 0.955797525 | -0.264696252 | 0.396037988 | 0.879255256 | -0.259987511 | -0.746441091 | -0.61256199  | 0.52469785   | -0.850812152 | 0.028475391  | 0.171805847       | -0.278587957      | 0.009323916       | -0.044139517   | 0.016644083    | -0.078400482   | -0.002848   | -0.046509   | 0.009805    | 0     |
| -0.060826806 | -0.287624644 | 0.95580969   | -0.26442447  | 0.397496032  | 0.878678897  | -0.059906299 | -0.288264403 | 0.955675086 | -0.265247511 | 0.396729437 | 0.878777282 | 0.260738265  | -0.506384611 | -0.821942932 | 0.162231636  | -0.868545219 | 0.468305562  | 0.066946532       | -0.358414006      | 0.193251046       | 0.034322951    | -0.031938411   | -0.088189996   | 0.02162     | -0.019908   | 0.002586    | 0     |
| -0.069062529 | -0.288161093 | 0.955088243  | -0.263882463 | 0.405076711  | 0.87537358   | -0.060623123 | -0.288152188 | 0.955663724 | -0.265685523 | 0.397368956 | 0.878355916 | 0.06959041   | -0.568321684 | -0.819858304 | 0.812688923  | -0.011961465 | 0.582575006  | 0.219561499       | -0.00323159       | 0.157392377       | 0.059482981    | 0.011454942    | -0.16790262    | 0.042848    | 0.006716    | -0.002917   | 0     |


- **training**
```
import xgboost as xgb
train_x, train_y, test_x, test_y = LoadData()
x, y = LoadVaildData()

D_train = xgb.DMatrix(train_x, label=train_y)
D_test = xgb.DMatrix(test_x, label=test_y)
D_validation = xgb.DMatrix(x, label=y)

param = {
    'objective':'multi:softprob',
    'eta': 0.3,
    'max_depth': 10,
    'num_class': 2
}

start_time = time.time()
model = xgb.train(param, D_train, 100)
end_time = time.time()

test_preds = model.predict(D_test)
test_best_preds = np.asarray([np.argmax(line) for line in test_preds])
vali_preds = model.predict(D_validation)
vali_best_preds = np.asarray([np.argmax(line) for line in vali_preds])

# dump model
 model.dump_model('./raw_model.txt', dump_format='txt')
print("Cost time: " + str(int(end_time - start_time)))
print(classification_report(test_y, test_best_preds, target_names=["??????", "??????"]))
print(classification_report(y, vali_best_preds, target_names=["??????", "??????"]))
```

- **rewrite the prediction in c++**
```
#include "utils/Tools.h"
#include "models/XgboostDetector.h"
std::string model_path = "raw_model.txt";
XgboostDetector xgboostDetector = XgboostDetector(model_path);
StopDetection &stopDetection = xgboostDetector;
bool is_stop = stopDetection.IsStopping(input_data);
```
