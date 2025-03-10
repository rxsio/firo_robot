from typing import TypedDict, List, Dict

class Field(TypedDict):
    field: str
    value: str | int | float | bool
    tags: Dict[str, str]

def write_measurements(measurement: str, fields: List[Field]) -> None:
    pass

