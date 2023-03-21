# Ouster-Lidar Data extraction

This repo contains sample code to 
1. Connect to the Ouster lidar
2. Collect data from the Ouster lidar
3. Publish the data using DDS protocol
4. Subscribe to the data using DDS protocol

### Binaries
This repo creates 2 binary applications, one for publisher and another one for subscriber. The source code for this binaries is present in the application folder.

### To run the publisher
`cargo r --bin publisher`

### To run the subscriber
`cargo r --bin subscriber`



