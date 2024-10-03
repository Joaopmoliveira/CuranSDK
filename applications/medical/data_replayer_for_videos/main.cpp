#include <iostream>

char volume_vert[] = R"(<PlusConfiguration version=\"2.1\">
  <DataCollection StartupDelaySec=\"1.0\" >
    <DeviceSet 
      Name=\"PlusServer: Replay data for video reproduction\"
      Description=\"replay recording.\" />
    <Device
      Id=\"TrackedVideoDevice\"
      Type=\"SavedDataSource\"
      SequenceFile=\"fCal_Test_Calibration_3NWires.igs.mha\"
      UseData=\"IMAGE_AND_TRANSFORM\"
      UseOriginalTimestamps=\"TRUE\"
      RepeatEnabled=\"TRUE\" >
      <DataSources>
        <DataSource Type=\"Video\" Id=\"Video\" />
      </DataSources>
      <OutputChannels>
        <OutputChannel Id=\"TrackedVideoStream\" VideoDataSourceId=\"Video\" />
      </OutputChannels>
    </Device>
  </DataCollection>

<CoordinateDefinitions>
    <Transform From=\"Image\" To=\"Flange\"
      Matrix=\"
        1	0	0	0
        0	1	0	0
        0	0	1	0
        0	0	0	1\" />
  </CoordinateDefinitions>

  <PlusOpenIGTLinkServer MaxNumberOfIgtlMessagesToSend=\"1\" MaxTimeSpentWithProcessingMs=\"50\" ListeningPort=\"18944\" SendValidTransformsOnly=\"true\" OutputChannelId=\"TrackedVideoStream\">
    <DefaultClientInfo>
      <MessageTypes>
        <Message Type=\"IMAGE\" />
      </MessageTypes>
      <ImageNames>
        <Image Name=\"Image\" EmbeddedTransformToFrame=\"Base\" />
      </ImageNames>
    </DefaultClientInfo>
  </PlusOpenIGTLinkServer>

</PlusConfiguration>)";

int main(int argc, char* argv[]){
    if(argc != 2){
        std::cout << "to call this executable, you need to provide four arguments:\n"
                  << "DataReplayer joint_recording.txt plus_configuration_file.xml plus_recording_file.xml";
    }



    return 0;
}