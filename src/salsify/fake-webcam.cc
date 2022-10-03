/* -*-mode:c++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

/* Copyright 2013-2018 the Alfalfa authors
                       and the Massachusetts Institute of Technology

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:

      1. Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.

      2. Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <iostream>
#include <chrono>
#include <thread>

#include "yuv4mpeg.hh"
#include "paranoid.hh"
#include "encoder.hh"
#include "decoder.hh"
#include "packet.hh"
#include <cmath>

using namespace std;

void usage( const char *argv0 )
{
  cerr << "Usage: " << argv0 << " INPUT FPS" << endl;
}

int main( int argc, char *argv[] )
{
  /* check the command-line arguments */
  if ( argc < 1 ) { /* for sticklers */
    abort();
  }

  if ( argc != 4 ) {
    usage( argv[ 0 ] );
    return EXIT_FAILURE;
  }
  cout << "in program" << endl;

  /* open the YUV4MPEG input */
  YUV4MPEGReader input { argv[ 1 ] };
  cout << "finish reading input" << endl;

  /* parse the # of frames per second of playback */
  unsigned int frames_per_second = paranoid::stoul( argv[ 2 ] );
  int quant_index = paranoid::stoul(argv[3]);
  /* open the output */
  FileDescriptor stdout { STDOUT_FILENO };

  const auto interval_between_frames = chrono::microseconds( int( 1.0e6 / frames_per_second ) );

  // auto next_frame_is_due = chrono::system_clock::now();

  uint16_t width = 1280, height = 768;

  Encoder base_encoder { width, height, false /* two-pass */, REALTIME_QUALITY };
  Decoder decoder {width, height};

  int sum = 0;
  int tot = 0;
  double tot_PSNR = 0;
  // bool initialized = false;
  while ( true ) {
    /* wait until next frame is due */
    // this_thread::sleep_until( next_frame_is_due );
    // next_frame_is_due += interval_between_frames;
    cout << "new frame" << endl;
    /* get the next frame to send */
    const Optional<RasterHandle> raster = input.get_next_frame();
    cout << "gettting new frame" << endl;
    if ( not raster.initialized() ) {
      break; /* eof */
    }
    auto output = base_encoder.encode_with_quantizer(raster.get(), quant_index);
    cout << output.size() << endl;

    FragmentedFrame ff { 0, 0, 0,
                           (uint32_t)tot,
                           5,
                           output};
    string s_frame = ff.frame();

    Chunk chunk(s_frame);
    auto output_raster = decoder.parse_and_decode_frame(chunk);
    cout << "output hash: " << output_raster.get().hash() << endl;
    const VP8Raster &input_vp8raster = raster.get();

    const VP8Raster &output_vp8raster = output_raster.get();
    double MSE = 0;
    
    cout << output_vp8raster.Y().at(0,0) << endl;
    for (int i = 0; i < width; i++) {
      for (int j = 0; j < height; j++) {
        double I = input_vp8raster.Y().at(i, j), K = output_vp8raster.Y().at(i, j);
        MSE += 1.0 * (I - K) * (I - K);
      }
    }
    for (int i = 0; i < width / 2; i++) {
      for (int j = 0; j < height / 2; j++) {
        double I = input_vp8raster.U().at(i, j), K = output_vp8raster.U().at(i, j);
        MSE += 1.0 * (I - K) * (I - K);
        I = input_vp8raster.U().at(i, j), K = output_vp8raster.U().at(i, j);
        MSE += 1.0 * (I - K) * (I - K);

      }
    }
    MSE /= (width * height * 1.5);
    
    double PSNR = 20 * log10(255) - 10 * log10(MSE);

    /* send the file header if we haven't already */
    // if ( not initialized ) {
    //   auto file_header = YUV4MPEGHeader( raster.get() );
    //   file_header.fps_numerator = frames_per_second;
    //   file_header.fps_denominator = 1;
    //   stdout.write( file_header.to_string() );
    //   initialized = true;
    // }
    sum += (int)output.size();
    tot++;
    tot_PSNR += PSNR;
    cout << "PSNR: " << PSNR << endl;

    /* send the frame */
    // YUV4MPEGFrameWriter::write( raster.get(), stdout );
  }
  cout << "totframes: " << tot << endl;
  cout << "average bpp: " << (1.0 * sum * 8 / tot / width / height) << endl; 
  cout << "totsum: " << sum << " bytes"<< endl;
  cout << "average PSNR: " << tot_PSNR / tot << endl;

}
