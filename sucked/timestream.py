


import boto3
import time

from botocore.config import Config


session = boto3.Session()




write_client = session.client('timestream-write', config=Config(read_timeout=20, max_pool_connections = 5000, retries={'max_attempts': 10})) 


# write_client.create_database(DatabaseName="Icebreaker4") 

# retention_properties = {
#     'MemoryStoreRetentionPeriodInHours': 24,
#     'MagneticStoreRetentionPeriodInDays': 365*50
# }

# write_client.create_table(DatabaseName="Icebreaker4", TableName="iccp4",
#                              RetentionProperties=retention_properties)


print("Writing records") 
current_time = str(int(round(time.time() * 1000)))


dimensions = [ 
   {'Name': 'region', 'Value': 'us-east-1'}, 
   {'Name': 'az', 'Value': 'az1'}, 
   {'Name': 'hostname', 'Value': 'host1'} 
] 

cpu_utilization = { 
   'Dimensions': dimensions, 
   'MeasureName': 'cpu_utilization', 
   'MeasureValue': '13.5', 
   'MeasureValueType': 'DOUBLE', 
   'Time': current_time 
} 

memory_utilization = { 
   'Dimensions': dimensions, 
   'MeasureName': 'memory_utilization', 
   'MeasureValue': '40', 
   'MeasureValueType': 'DOUBLE', 
   'Time': current_time 
} 

records = [cpu_utilization, memory_utilization] 

try: 
   result = write_client.write_records(DatabaseName="Icebreaker4", TableName="iccp4", 
                                      Records=records, CommonAttributes={}) 
   print("WriteRecords Status: [%s]" % result['ResponseMetadata']['HTTPStatusCode']) 
except Exception as err: 
   print("Error:", err)


