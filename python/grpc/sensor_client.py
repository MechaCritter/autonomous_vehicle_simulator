# stream_client.py
import grpc
import stream_pb2
import stream_pb2_grpc

def run():
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = stream_pb2_grpc.DataStreamerStub(channel)
        for data_point in stub.StreamData(stream_pb2.Empty()):
            print(f"Received: value={data_point.value}, timestamp={data_point.timestamp}")

if __name__ == '__main__':
    run()
