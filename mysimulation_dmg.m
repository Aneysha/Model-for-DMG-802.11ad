Total_bytes= 2764800;
Total_bits=2764800*8;
msdu_bytes = 7920;
msduLength = 7920*8; %MSDU length in bits.
maxNumPackets =ceil(Total_bytes/msdu_bytes);
padZeros = msdu_bytes-mod(Total_bytes,msdu_bytes);
Total_bits = Total_bits + padZeros;
MCS = '3';
cfgDMG = wlanDMGConfig('MCS','3');
cfgDMG.PSDULength = 7920;
fs = wlanSampleRate(cfgDMG);
fc = 60e9;

% Create a TGay channel object
tgayChan = wlanTGayChannel;
tgayChan.SampleRate                = fs;
tgayChan.CarrierFrequency          = fc;
tgayChan.Environment               = 'Open area hotspot';
tgayChan.RandomRays                = true;
tgayChan.IntraClusterRays          = true;
tgayChan.RandomStream              = 'Global stream';
tgayChan.ReceiveArrayVelocitySource= 'Custom';
tgayChan.ReceiveArrayVelocity      = [0; 1; 0];
tgayChan.TransmitArray.Size        = [4 4];
tgayChan.TransmitArrayPosition     = [0; 0; 6];     % Meters
tgayChan.TransmitArrayOrientation  = [0; 270; 0];   % Degrees
tgayChan.ReceiveArray.Size         = [4 4];
tgayChan.ReceiveArrayPosition      = [6; 6; 1.5];   % Meters
tgayChan.ReceiveArrayOrientation   = [90; 0; 0];    % Degrees
tgayChan.BeamformingMethod         = 'Maximum power ray';
tgayChan.NormalizeImpulseResponses = true;

ppm = 20;                 % Clock accuracy for the Carrier Frequency Offset (ppm)
freqOffset = ppm*1e-6*fc; % Carrier frequency offset (Hz)
delay = 500;              % Sample to delay the waveform. This value can be changed.
zeroPadding = 100;        %For trailing zeros to simulate channel delay

snrRanges = {-2.0:0.5:0.5, ...  % MCS 1
    0.0:1.0:5.0, ...   % MCS 2
    1.0:1.2:7.0, ...   % MCS 3
    2.0:1.2:8.0, ...   % MCS 4
    2.0:1.3:8.5, ...   % MCS 5
    2.5:1.3:9.0, ...   % MCS 6
    4.0:1.3:10.5, ...  % MCS 7
    5.0:1.5:12.5,...   % MCS 8
    5.5:1.5:13.0, ...  % MCS 9
    7.0:1.5:14.5, ...  % MCS 9.1
    8.0:1.8:17.0, ...  % MCS 10
    10.0:2.0:20.0, ... % MCS 11
    12.0:2.0:22.0, ... % MCS 12
    12.0:2.0:22.0, ... % MCS 12.1
    14.0:2.0:24.0, ... % MCS 12.2
    16.0:2.5:28.5, ... % MCS 12.3
    17.0:2.5:29.5, ... % MCS 12.4
    17.0:2.5:29.5, ... % MCS 12.5
    20.0:2.5:32.5};    % MCS 12.6

MCS_Valid = string(sort([1:12 9.1 12.1:0.1:12.6]));
SNR_Num = numel(snrRanges{1}); % Number of SNR points
packetErrorRate = zeros(1,SNR_Num);
Ngi = 64; % GI length defined as per the standard

ind = wlanFieldIndices(cfgDMG);
snr = snrRanges{MCS==MCS_Valid}; % SNR points to simulate from MCS
% bit_error_array = cell(maxNumPackets,SNR_Num);
bit_error_array = zeros(maxNumPackets,SNR_Num);
dir_path = 'F:\Master Thesis\Wireless\SOFTX_2019_390-master (1)\SOFTX_2019_390-master\Codigo_final_PHY_802_11_p_v1.1\Codigo_final_PHY_802_11_p_RICIAN\code\Input_data\';
files = dir([dir_path '*.png']);
parfor isnr = 1:SNR_Num
    % isnr = 1%:SNR_Num
    k = 6;    %:length(files)
    file_name = files(k).name;
    imgTX=imread([dir_path file_name]); %read image
    origSize = size(imgTX); %[720,1280,3]
    flatImage = imgTX(:); %convert to 1D vector (not a binary string)
    maxNumPackets =ceil(length(flatImage)/msdu_bytes);
    zeros_4_padding = zeros(msdu_bytes*maxNumPackets-length(flatImage),1);
    flatImage = [flatImage;zeros_4_padding];
    
    
    % Set simulation parameters
    numPacketErrors = 0;
    numPkt = 1; % Index of the transmitted packet
    fprintf('%i\n',isnr );
    bit_error_rate_i = zeros(maxNumPackets,1);
    for i= 0:maxNumPackets-1
        fprintf('Packet%i\n',i);
        
        framebody = (flatImage(i*msdu_bytes+1:msdu_bytes*(i+1),:));
        psdu = double((reshape(de2bi(framebody, 8)', [], 1)));
        waveform = wlanWaveformGenerator(psdu,cfgDMG);
        
        % Add delay and trailing zeros
        tx = [zeros(500,1); waveform; zeros(100,1)];
        
        % Transmit through a TGay channel. Reset the channel for a
        % different realization per packet.
%                 reset(tgayChan);
%                 chanOut = tgayChan(tx);
        %          Packet_TX= transpose(tx);
        %        [Packet_RX,H] = RICIANChannel(Packet_TX,fs);
        % Add noise
        rx = awgn(tx,snr(isnr));
        
        %Receiver
        %Add Channel Frequency Offset
        rx = helperFrequencyOffset(rx,fs,freqOffset);
        
        % Packet detection
        threshold = 0.03; % Good for low SNRs
        OFFset_startpacket = dmgPacketDetect(rx,0,threshold);
        if isempty(OFFset_startpacket) % If empty no STF detected; packet error
            numPacketErrors = numPacketErrors+1;
            numPkt = numPkt+1;
            continue; % Go to next loop iteration
        end
        
        % Coarse Frequency offset correction
        stf = rx(OFFset_startpacket+(ind.DMGSTF(1):ind.DMGSTF(2)));
        coarse_estfreq = dmgCFOEstimate(stf);
        rx = helperFrequencyOffset(rx,fs,-coarse_estfreq);
        
        % Channel estimate
        preamble = rx(OFFset_startpacket+1:OFFset_startpacket+ind.DMGHeader(2),:);
        [symbol_Offset,chanEst] = dmgTimingAndChannelEstimate(preamble);
        startOffset = OFFset_startpacket+symbol_Offset;
        
        % If not enough samples to decode detected data field start,
        % then assume synchronization error and packet error
        if (startOffset+ind.DMGData(2))>size(rx,1)
            numPacketErrors = numPacketErrors+1;
            numPkt = numPkt+1;
            continue; % Go to next loop iteration
        end
        
        % Noise estimation 
        stf = rx(OFFset_startpacket+(ind.DMGSTF(1):ind.DMGSTF(2)));
        noise_VarEst = dmgSTFNoiseEstimate(stf);
        
        % Extract data field
        rxData = rx(startOffset+((ind.DMGData(1)+Ngi):ind.DMGData(2)));
        
        % Linear frequency domain equalization
        rx_EqBlks = dmgSingleCarrierFDE(rxData,chanEst,noise_VarEst);
        
        % Unique word phase tracking
        rx_EqBlks = dmgUniqueWordPhaseTracking(rx_EqBlks);
        
        % Discard GI from blocks
        rxDataSym = rx_EqBlks(1:end-Ngi,:);
        
        % Recover the transmitted PSDU
        dataDecode = wlanDMGDataBitRecover(rxDataSym,noise_VarEst,cfgDMG);
        
        bit_error = 0;
        for j =1 :length(dataDecode)
            if  dataDecode(j)~= psdu(j)
                bit_error = bit_error + 1;
            end

        end
            %         bit_error_array{i+1,isnr} = bit_error;
        bit_error_rate_i(i+1) = bit_error;
        
%         bit_error_array(i+1,isnr) = bit_error;        
        % Determine if any bits are in error, i.e. a packet error
        packetError = any(biterr(psdu,dataDecode));
        numPacketErrors = numPacketErrors+packetError;
        numPkt = numPkt+1;
    end
    bit_error_array(:,isnr) = bit_error_rate_i;
        
    
    
    % Calculate packet error rate (PER) at SNR point
    packetErrorRate(isnr) = numPacketErrors/(numPkt-1);
    %     disp(join(["     MCS:" cfgDMG.MCS ", SNR " ...
    %         num2str(snr(isnr)) " completed after " ...
    %         num2str(numPkt-1) " packets, PER: " ...
    %         num2str(packetErrorRate(imcs,isnr))],""));
    
end

save('AWGN_MCS3_BER','bit_error_array','snr');
save('AWGN_MCS3_PER','packetErrorRate','snr');
