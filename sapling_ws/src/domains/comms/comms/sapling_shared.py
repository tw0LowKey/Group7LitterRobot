import struct
from typing import Optional

def encodePacket(protocol, cmdName, *args) -> Optional[dict]:
	""" Finds a command in the protocol and packs the provided args using its struct_format """

	# Search for the correct command details in the protocol
	for tempCmdId, tempCmdInfo in protocol.items():
		if tempCmdInfo.get("name") == cmdName:
			cmdId = int(tempCmdId)
			structFormat = tempCmdInfo.get("structFormat")

			# Pack the command ID followed by the provided payload values
			if structFormat:
				encodedPacket = struct.pack(structFormat, cmdId, *args)

				return encodedPacket.hex()

			break

	return None

def decodePacket(packet: bytearray, protocol: dict) -> dict:
	# Get the binary data of the packet and the associated command ID, name and info
	packet = bytes.fromhex(packet.decode("utf-8"))
	cmdId = packet[0]
	cmdInfo = protocol.get(str(cmdId), {})
	cmdName = cmdInfo.get("name", "unknown")

	# Unpack the binary data
	unpackedPacket = struct.unpack(cmdInfo["structFormat"], packet)

	# Zip the protocol keys together with the payload values
	payload = {}
	for i, key in enumerate(cmdInfo.get("keys", [])):
		if key != "cmdId":
			val = unpackedPacket[i]

			if isinstance(val, bytes): # NOTE: See if this can be removed
				val = val.decode("ascii")

			payload[key] = val

	decodedPacket = { "cmdName": cmdName, "payload": payload }

	return decodedPacket
