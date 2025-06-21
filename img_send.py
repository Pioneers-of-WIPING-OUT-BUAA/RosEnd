import os
import json
import requests
import time

IMG_SAVE_DIR = "/tmp/img"


def upload_loop():
    last_uploaded = None
    while True:
        time.sleep(5)
        try:
            files = sorted(os.listdir(IMG_SAVE_DIR), reverse=True)
            if not files:
                continue
            latest_file = os.path.join(IMG_SAVE_DIR, files[0])
            if latest_file == last_uploaded:
                continue
            
            pos_list = (latest_file.split(".")[0]).split("_")
            pos = [float(pos_list[1]), float(pos_list[2]), float(pos_list[3])]

            img_send(latest_file, pos)
            print(f"[master_node] uploaded image: {latest_file}")
            last_uploaded = latest_file
        except Exception as e:
            print(f"[master_node] upload_loop error: {e}")


def img_send(filename, pos=[0, 0, 0]):
    """
    result: 检测结果
        0: 未检测到异常
        1: 检测到火焰
        2: 检测到烟雾
        3: 检测到陌生人
    """
    try:
        url = 'http://10.212.30.237:8000/api/detect/upload'

        if not os.path.exists(filename):
            print(f"Error: File {filename} does not exist")
            return 0

        if not isinstance(pos, list) or len(pos) != 3:
            print(f"Warning: Invalid position format {pos}, using default [0, 0, 0]")
            pos = [0, 0, 0]

        with open(filename, "rb") as f:
            files = {"file": (os.path.basename(filename), f, "image/jpeg")}
            data = {"pos": json.dumps(pos)}
            res = requests.post(url=url, files=files, data=data)

        res_data = json.loads(res.text)

        exist_fire, exist_smoke, exist_stranger, exist_rubbish = False, False, False, False
        if res_data.get("fire", False):
            exist_fire = True
        if res_data.get("smoke", False):
            exist_smoke = True
        if res_data.get("stranger", False):
            exist_stranger = True
        if res_data.get("rubbish", False):
            exist_rubbish = True

        result_dict= {
          "pos": pos,
          "time": time.strftime('%Y-%m-%d %H:%M:%S'),
          "exist_fire": exist_fire,
          "exist_smoke": exist_smoke,
          "exist_stranger": exist_stranger,
          "exist_rubbish": exist_rubbish,
          "filename": filename
        }

        # folder = f"/tmp/report/{time.strftime('%Y-%m-%d')}"
        # if not os.path.exists(folder):
        #     os.makedirs(folder)
        # with open(f"{folder}/detect_result.json", "w") as f:
        #     f.write(json.dumps(result_dict))

        return result_dict

    except Exception as e:
        print(f"Error in img_send: {e}")
        return 0
      

if __name__ == "__main__":
    upload_loop()