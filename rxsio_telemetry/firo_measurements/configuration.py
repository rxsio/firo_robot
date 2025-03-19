from pydantic import BaseModel, Field
from typing import List, Dict, Optional 

class MeasurementField(BaseModel):
    field: str
    value: str
    tags: Dict[str, str] = {}
    for_each: Optional[str] = None


class Measurement(BaseModel):
    name: str
    measurement_fields: List[MeasurementField] = Field(
        alias="fields", default=[])
    
class Topic(BaseModel):
    name: str
    type: Optional[str] = None
    measurements: List[Measurement] = []

class Influx(BaseModel):
    url: str
    token: str
    bucket: str
    org: str

class Outputs(BaseModel):
    influx: Optional[Influx] = None


class Configuration(BaseModel):
    outputs: Outputs
    topics: List[Topic] = []


def validate_configuration(**args) -> Optional[Configuration]:
    return Configuration(**args)