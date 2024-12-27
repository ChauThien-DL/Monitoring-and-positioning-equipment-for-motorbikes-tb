import requests
import time
import joblib
import pandas as pd
from datetime import datetime, timezone
import pytz 
import pymongo

client = pymongo.MongoClient("mongodb+srv://doanck:123@doanck.686bp.mongodb.net/?retryWrites=true&w=majority&appName=doanck") 
db = client["MyDatabase"]  
collection = db["Warning"]  

url = "http://52.74.168.92:8888/get_dataTrain"

headers = {
    "X-API-Key": "deadline"
}

model_path = 'decision_tree_model.pkl'

def predict_new_data(input_data):
    """
    Dự đoán lớp cho dữ liệu mới dựa trên mô hình đã huấn luyện.

    :param input_data: Danh sách giá trị đặc trưng [Mode_anti-theft, Key, Vibration, Acceleration, Distance, Speed]
    :return: Lớp dự đoán
    """
    loaded_model = joblib.load(model_path)

    column_names = ['Mode_anti-theft', 'Key', 'Vibration', 'Acceleration', 'Distance', 'Speed']
    input_df = pd.DataFrame([input_data], columns=column_names)

    prediction = loaded_model.predict(input_df)
    return prediction


def get_next_id(col):
    max_id = col.find_one(sort=[("_id", pymongo.DESCENDING)])  
    if max_id is not None:
        next_id = int(max_id["_id"]) + 1 
    else:
        next_id = 1  
    return next_id

def get_vietnam_time():
    utc_now = datetime.now(timezone.utc)  
    vietnam_tz = pytz.timezone("Asia/Ho_Chi_Minh")  
    vietnam_time = utc_now.replace(tzinfo=pytz.utc).astimezone(vietnam_tz)  
    return vietnam_time.strftime("%Y-%m-%dT%H:%M:%SZ")

while(True):

    response = requests.get(url, headers=headers)

    data = response.json()
    print(data)

    dataTrain = list(data.values())[-6:]

    print(dataTrain)

    pre = predict_new_data(dataTrain)
 
    if pre[0] == 1:
        predict_data = "Mất"
    else:
        predict_data = "Không"

    print('Dự đoán: ', predict_data)

    new_document = {
        "_id": get_next_id(collection),  
        "datetime": get_vietnam_time(),  
        "predict": predict_data
    }
    collection.insert_one(new_document)
    print(f"Data sent: {new_document}")

    time.sleep(60)