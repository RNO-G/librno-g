/** Python Bindings for librno-g 
 *  
 *  Uses pybind11 (so this file is really C++) 
 *
 *  You'll obviously need pybind11 to compile this (on fedora/EL, dnf install pybind11-devel python3-pybind11) 
 *
 *  Enjoy. 
 *
 * */ 

extern "C" { 
#include "rno-g.h" 
}



#include <pybind11/pybind11.h> 
#include <pybind11/pytypes.h> 
#include <pybind11/stl.h> 
#include <pybind11/numpy.h> 
#include <stdlib.h> 

namespace py = pybind11;  



struct py_rno_g_file_handle 
{

  rno_g_file_handle_t h; 

  py_rno_g_file_handle(const std::string & name, const std::string & mode)
  {
    if (rno_g_init_handle(&h, name.c_str(), mode.c_str()) ) throw std::exception(); 
  }
  ~py_rno_g_file_handle() 
  {
    rno_g_close_handle(&h); 
  }
}; 





PYBIND11_MODULE(rno_g,m) 
{

  m.doc() = "Python bindings for librno-g"; 

  py::class_<py_rno_g_file_handle>(m,"FileHandle")
    .def(py::init<const std::string & , const std::string & >()); 

#define FLD(x) .def_readonly(#x,&rno_g_header_t :: x)
  py::class_<rno_g_header_t>(m,"Header")
    FLD(event_number)
    FLD(run_number)
    FLD(sys_clk)
    FLD(pps_count)
    FLD(station_number)
    FLD(readout_time_secs)
    FLD(readout_time_nsecs)
    FLD(readout_elapsed_nsecs)
    FLD(sysclk_last_pps)
    FLD(sysclk_last_last_pps)
    FLD(raw_tinfo)
    FLD(raw_evstatus)
    FLD(flags)
    FLD(trigger_type)
    FLD(radiant_nsamples)
    .def_property_readonly("radiant_start_windows", [](rno_g_header_t & header) {  return reinterpret_cast<std::array<std::array<uint8_t, 2>, RNO_G_NUM_RADIANT_CHANNELS>&>(header.radiant_start_windows);})
    .def("read", [](rno_g_header_t & header, py_rno_g_file_handle & handle) { return rno_g_header_read(handle.h, &header); })
    .def("write", [](rno_g_header_t & header, py_rno_g_file_handle & handle) { return rno_g_header_write(handle.h, &header);} )
    .def("__init__",[](rno_g_header_t & header) { memset(&header,0,sizeof(header)); })
  ;


#undef FLD
#define FLD(x) .def_readonly(#x,&rno_g_waveform_t :: x)
  py::class_<rno_g_waveform_t>(m,"Waveform")
    FLD(event_number)
    FLD(run_number)
    FLD(radiant_nsamples)
    FLD(lt_nsamples)
    FLD(station)
   // .def_property_readonly("radiant_waveforms", []( rno_g_waveform_t & wf) { return reinterpret_cast<std::array<std::array<int16_t, RNO_G_MAX_RADIANT_NSAMPLES>, RNO_G_NUM_RADIANT_CHANNELS>&> (wf.radiant_waveforms);})
    .def_property_readonly("radiant_waveforms", []( rno_g_waveform_t & wf) {
        int sz = sizeof(wf.radiant_waveforms[0][0]); 
        return py::array(py::buffer_info(
              wf.radiant_waveforms, 
              sz, 
              py::format_descriptor<int16_t>::value, 
              2, 
              { RNO_G_NUM_RADIANT_CHANNELS, RNO_G_MAX_RADIANT_NSAMPLES }, 
              {sz * RNO_G_MAX_RADIANT_NSAMPLES,sz})); })
    .def("read", [](rno_g_waveform_t & waveform, py_rno_g_file_handle & handle) { return rno_g_waveform_read(handle.h, &waveform); } )
    .def("write", [](rno_g_waveform_t & waveform, py_rno_g_file_handle & handle) { return rno_g_waveform_write(handle.h, &waveform); } )
    .def("__init__",[](rno_g_waveform_t & wf) { memset(&wf,0,sizeof(wf)); })
  ;


} 





