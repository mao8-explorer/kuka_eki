
from dataclasses import dataclass
import xml.etree.ElementTree as ET
from typing import Tuple, Union
from enum import IntEnum

@dataclass
class Axis:
    a1: float = 0.0
    a2: float = 0.0
    a3: float = 0.0
    a4: float = 0.0
    a5: float = 0.0
    a6: float = 0.0
    e1: float = 0.0

    def to_xml(self, root: ET.Element) -> ET.Element:
        element = ET.SubElement(
            root,
            "Axis",
            {
                "A1": str(self.a1),
                "A2": str(self.a2),
                "A3": str(self.a3),
                "A4": str(self.a4),
                "A5": str(self.a5),
                "A6": str(self.a6),
                # "E1": str(self.e1),
            },
        )
        return element


@dataclass
class Pos:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    # s: int = 0
    # t: int = 0
    e1: float = 0.0

    def to_xml(self, root: ET.Element) -> ET.Element:
        element = ET.SubElement(
            root,
            "Cart",
            {
                "X": str(self.x),
                "Y": str(self.y),
                "Z": str(self.z),
                "A": str(self.a),
                "B": str(self.b),
                "C": str(self.c),
                # "S": str(self.s),
                # "T": str(self.t),
                # "E1": str(self.e1),
            },
        )
        return element

    def __iter__(self):
        return iter((self.x, self.y, self.z, self.a, self.b, self.c))
    


@dataclass
class PosTgtServer:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    

@dataclass
class AxisE1:
    e1: float = 0.0

    def to_xml(self, root: ET.Element) -> ET.Element:
        element = ET.SubElement(
            root,
            "Axis",
            {
                "E1": str(self.e1),
            },
        )
        return element
    


@dataclass
class RobotState:
    axis: Axis = Axis()
    pos: Pos = Pos()
    posTgtServer: PosTgtServer = PosTgtServer()

    @classmethod
    def from_xml(cls, xml: bytes):
        root: ET.Element = ET.fromstring(xml)
        attrib: dict[str, str] = root.find("Pos").attrib
        pos = Pos(
            attrib["X"],
            attrib["Y"],
            attrib["Z"],
            attrib["A"],
            attrib["B"],
            attrib["C"],
            # attrib["S"],
            # attrib["T"],
        )
        attrib = root.find("Axis").attrib
        axis = Axis(
            attrib["A1"],
            attrib["A2"],
            attrib["A3"],
            attrib["A4"],
            attrib["A5"],
            attrib["A6"],
            attrib["E1"],
        )
        
        attrib = root.find("PosTgt").attrib
        posTgtServer = PosTgtServer(
            attrib["X"],
            attrib["Y"],
            attrib["Z"],
        )

        return RobotState(axis, pos, posTgtServer) 

    def to_xml(self) -> bytes:
        root: ET.Element = ET.Element("RobotState")
        self.axis.to_xml(root)
        self.pos.to_xml(root)
        return ET.tostring(root)



class CommandType(IntEnum):
    PTP_AXIS = 1
    PTP_CART = 2


@dataclass
class CommandBuffersize:
    buffersize: int = 0
    @classmethod
    def from_xml(cls, xml: bytes):
        root: ET.Element = ET.fromstring(xml)
        attrib: dict[str, str] = root.find("RobotCommand").attrib
        buffersize = int(attrib["Size"])
        return buffersize

@dataclass
class RobotCommand:
    command_type: CommandType
    target: Union[Axis, Pos]
    velocity_scaling: int

    def to_xml(self) -> bytes:
        root = ET.Element("RobotCommand")
        ET.SubElement(root, "Type").text = str(self.command_type.value)
        self.target.to_xml(root)
        Pos().to_xml(root) if isinstance(self.target, Axis) else Axis().to_xml(root)
        ET.SubElement(root, "Velocity").text = str(self.velocity_scaling)
        return ET.tostring(root)
    
@dataclass
class E1RobotCommand:
    target: AxisE1

    def to_xml(self) -> bytes:
        root = ET.Element("RobotCommand")
        self.target.to_xml(root)
        return ET.tostring(root)
    

@dataclass
class SpeedCommand:
    velocity_scaling: int

    def to_xml(self) -> bytes:
        root = ET.Element("RobotCommand")
        ET.SubElement(root, "Velocity").text = str(self.velocity_scaling)
        return ET.tostring(root)
    
