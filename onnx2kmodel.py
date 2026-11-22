import os, glob
import numpy as np
import cv2
import nncase
import nncase_kpu

# ====== 改这几行 ======
MODEL = r"D:\工创赛\runs\detect\train8\weights\best.onnx"
DATASET = r"D:\工创赛\calib_images"
OUT     = r"D:\工创赛\gongchuangsai_yolo11n_det_320_v3.kmodel"
W, H    = 320, 320
MAX_SAMPLES = 58
PTQ_OPTION  = 0     # 0: NoClip+uint8/uint8（官方默认）；想更准可改 3: Kld+uint8/uint8
# =====================

def read_model_file(path):
    with open(path, "rb") as f:
        return f.read()

def generate_data(dataset_dir, w, h, max_samples):
    files = [f for f in glob.glob(os.path.join(dataset_dir, "*"))
             if f.lower().endswith((".jpg", ".jpeg", ".png"))][:max_samples]
    data = []
    for f in files:
        # ★ 用 np.fromfile + imdecode，解决中文路径 cv2.imread 读不了的问题
        buf = np.fromfile(f, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is None:
            print("skip:", f)
            continue
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)   # BGR->RGB
        img = cv2.resize(img, (w, h))                # 缩放到模型输入
        img = np.asarray(img, dtype=np.uint8)
        img = np.transpose(img, (2, 0, 1))           # HWC->CHW
        data.append([img[np.newaxis, ...]])          # [1,3,H,W]
    return data

def main():
    input_shape = [1, 3, H, W]

    compile_options = nncase.CompileOptions()
    compile_options.target       = "k230"
    compile_options.preprocess   = True
    compile_options.swapRB       = False             # 板端 pipeline 是 RGB
    compile_options.input_shape  = input_shape
    compile_options.input_type   = "uint8"
    compile_options.input_range  = [0, 1]            # uint8[0,255] -> [0,1]
    compile_options.mean         = [0, 0, 0]
    compile_options.std          = [1, 1, 1]
    compile_options.input_layout = "NCHW"

    compiler = nncase.Compiler(compile_options)
    compiler.import_onnx(read_model_file(MODEL), nncase.ImportOptions())

    ptq_options = nncase.PTQTensorOptions()
    if PTQ_OPTION == 0:
        ptq_options.calibrate_method = "NoClip"
        ptq_options.quant_type       = "uint8"
        ptq_options.w_quant_type     = "uint8"
    elif PTQ_OPTION == 3:
        ptq_options.calibrate_method = "Kld"
        ptq_options.quant_type       = "uint8"
        ptq_options.w_quant_type     = "uint8"
    else:
        raise ValueError("模板只写了 0 和 3")

    data = generate_data(DATASET, W, H, MAX_SAMPLES)
    print("calib samples:", len(data))
    ptq_options.samples_count = len(data)
    ptq_options.set_tensor_data(data)

    compiler.use_ptq(ptq_options)
    compiler.compile()

    with open(OUT, "wb") as f:
        f.write(compiler.gencode_tobytes())
    print("KModel OK ->", OUT)

if __name__ == "__main__":
    main()