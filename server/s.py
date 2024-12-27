from fastapi import FastAPI, HTTPException, Depends, Query
from typing import List
import uvicorn
from pydantic import BaseModel
import pymongo
from fastapi.security.api_key import APIKeyHeader
from datetime import datetime, timedelta
import pytz

myclient = pymongo.MongoClient("mongodb+srv://doanck:123@doanck.686bp.mongodb.net/?retryWrites=true&w=majority&appName=doanck")
mydb = myclient["MyDatabase"]
mycol1 = mydb["Full Data"]
mycol2 = mydb["GPS"]
mycol3 = mydb["DataTrain"]
mycol4 = mydb["Warning"]

app = FastAPI()

API_KEY = "deadline"

api_key_header = APIKeyHeader(name="X-API-Key")

def get_next_id(col):
    max_id = col.find_one(sort=[("_id", pymongo.DESCENDING)])
    if max_id is not None:
        next_id = int(max_id["_id"]) + 1
    else:
        next_id = 1
    return next_id

class F_Item(BaseModel):
    time: str
    device: str
    x:  int 
    y: int
    z: int
    Rotation: int
    temp: int
    o_key: int
    Vibration: int
    Mode_AntiTheft: int
    TotalDistance: float
    Vol_Acquy: float
    lat: float
    long: float
    Speed: int

@app.post("/post_fulldata")
async def update_fulldata(item: F_Item, api_key: str = Depends(api_key_header)):
    if (api_key != API_KEY):
        raise HTTPException(status_code=403, detail="API Key không hợp lệ")
    
    next_id = get_next_id(mycol1)

    mydict = {
        "_id": next_id,
        "datetime": item.time,
        "device_name": item.device,
        "x": item.x,
        "y": item.y,
        "z": item.z,
        "Rotation": item.Rotation,
        "temp": item.temp,
        "out_key": item.o_key,
        "Vibration": item.Vibration,
        "Mode_AntiTheft": item.Mode_AntiTheft,
        "TotalDistance": item.TotalDistance,
        "Vol_Acquy": item.Vol_Acquy,
        "lat": item.lat,
        "long": item.long,
        "speed": item.Speed
    }
    mycol1.insert_one(mydict)
    return {"OK"}

@app.get("/get_fulldata")
async def get_fulldata(api_key: str = Depends(api_key_header)):
    if api_key != API_KEY:
        raise HTTPException(status_code=403, detail="API Key không hợp lệ")

    latest_data = mycol1.find_one(sort=[("_id", -1)])
    
    if not latest_data:
        raise HTTPException(status_code=404, detail="Không có dữ liệu")
 
    result = {
        "id": str(latest_data["_id"]),
        "datetime": latest_data.get("datetime"),
        "device_name": latest_data.get("device_name"),
        "x": latest_data.get("x"),
        "y": latest_data.get("y"),
        "z": latest_data.get("z"),
        "Rotation": latest_data.get("Rotation"),
        "temp": latest_data.get("temp"),
        "out_key": latest_data.get("out_key"),
        "Vibration": latest_data.get("Vibration"),
        "Mode_AntiTheft": latest_data.get("Mode_AntiTheft"),
        "TotalDistance": latest_data.get("TotalDistance"),
        "Vol_Acquy": latest_data.get("Vol_Acquy"),
        "lat": latest_data.get("lat"),
        "long": latest_data.get("long"),
        "speed": latest_data.get("speed")
    }
    
    return result

class GPS_Item(BaseModel):
    time: str
    device: str
    lat: float
    long: float
    speed: float

@app.post("/post_gps")
async def update_gps(item: GPS_Item, api_key: str = Depends(api_key_header)):
    if api_key != API_KEY:
        raise HTTPException(status_code=403, detail="API Key không hợp lệ")
    
    next_id = get_next_id(mycol2)
    mydict = {
        "_id": next_id,
        "datetime": item.time,
        "device_name": item.device,
        "lat": item.lat,
        "long": item.long,
        "speed": item.speed
    }
    mycol2.insert_one(mydict)
    return {"OK"}

@app.get("/get_10_gps_data")
async def get_10_gps_data(api_key: str = Depends(api_key_header)):
    if api_key != API_KEY:
        raise HTTPException(status_code=403, detail="API Key không hợp lệ")

    gps_data = mycol2.find().sort("_id", pymongo.DESCENDING).limit(10)
    
    result = []
    for data in gps_data:
        result.append({
            "id": data["_id"],
            "datetime": data["datetime"],
            "device_name": data["device_name"],
            "lat": data["lat"],
            "long": data["long"],
            "speed": data["speed"]
        })
    result = sorted(result, key=lambda x: x["id"])
    
    return result


@app.get("/get_gps_history")
async def get_gps_history(date: str, api_key: str = Depends(api_key_header)):
    if api_key != API_KEY:
        raise HTTPException(status_code=403, detail="API Key không hợp lệ")
    
    try:
        date_obj = datetime.strptime(date, "%Y-%m-%d")
    except ValueError:
        raise HTTPException(status_code=400, detail="Định dạng ngày không hợp lệ. Vui lòng sử dụng định dạng YYYY-MM-DD.")

    start_of_day = date_obj.replace(hour=0, minute=0, second=0, microsecond=0)
    end_of_day = start_of_day + timedelta(days=1)

    gps_data = mycol2.find({"datetime": {"$gte": start_of_day.isoformat(), "$lt": end_of_day.isoformat()}})
    
    result = []
    for data in gps_data:
        result.append({
            "id": data["_id"],
            "datetime": data["datetime"],
            "device_name": data["device_name"],
            "lat": data["lat"],
            "long": data["long"],
            "speed": data["speed"]
        })
    
    return result

class DataTrainItem(BaseModel):
    time: str
    device: str
    Mode_AntiTheft: int
    key: int
    Vibration: int
    Acceleration: int
    Distance: int
    Speed: int

@app.post("/post_dataTrain")
async def update_dataTrain(item: DataTrainItem, api_key: str = Depends(api_key_header)):
    if (api_key != API_KEY):
        raise HTTPException(status_code=403, detail="API Key không hợp lệ")

    next_id = get_next_id(mycol3)
    mydict = {
        "_id": next_id,
        "time": item.time,
        "device": item.device,
        "Mode_AntiTheft": item.Mode_AntiTheft,
        "key": item.key,
        "Vibration": item.Vibration,
        "Acceleration": item.Acceleration,
        "Distance": item.Distance,
        "Speed": item.Speed
    }
    mycol3.insert_one(mydict)
    return {"OK"}

@app.get("/get_dataTrain")
async def get_dataTrain(api_key: str = Depends(api_key_header)):
    if api_key != API_KEY:
        raise HTTPException(status_code=403, detail="API Key không hợp lệ")

    latest_data = mycol3.find_one(sort=[("_id", -1)])

    if not latest_data:
        raise HTTPException(status_code=404, detail="Không có dữ liệu")

    result = {
        "id": str(latest_data["_id"]),
        "time": latest_data.get("time"),
        "device": latest_data.get("device"),
        "Mode_AntiTheft": latest_data.get("Mode_AntiTheft"),
        "key": latest_data.get("key"),
        "Vibration": latest_data.get("Vibration"),
        "Acceleration": latest_data.get("Acceleration"),
        "Distance": latest_data.get("Distance"),
        "Speed": latest_data.get("Speed")
    }
    return result

@app.get("/get_last_warning")
async def get_last_warning(api_key: str = Depends(api_key_header)):

    if api_key != API_KEY:
        raise HTTPException(status_code=403, detail="API Key không hợp lệ")

    last_warning = mycol4.find_one(sort=[("_id", -1)])  
    
    if not last_warning:
        raise HTTPException(status_code=404, detail="No warning found")

    result = {
        "id": last_warning["_id"], 
        "datetime": last_warning["datetime"],  
        "predict": last_warning["predict"] 
    }
    return result

@app.get("/")
async def root():
    return {"message": "Welcome to the API"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8888)
