
/** Python Bindings for librno-g 
 *  
 *  Uses pybind11 (so this file is really C++) 
 *
 *  Enjoy. 
 *
 * */ 

extern "C" { 
#include "rno-g.h" 
}



#include <pybind11/pybind11.h> 
#include <pybind11/pytypes.h> 
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
    FLD(trigger_number)
    FLD(run_number)
    FLD(trigger_mask)
    FLD(trigger_value)
    FLD(trigger_time)
    FLD(pps_count)
    FLD(station_number)
    FLD(trigger_type)
    FLD(flags)
    FLD(pretrigger_windows)
    FLD(radiant_nsamples)
    FLD(lt_nsamples)
    .def_property_readonly("radiant_start_windows", [](py::object & obj){  rno_g_header_t & h = obj.cast<rno_g_header_t&>(); return py::array ( {RNO_G_NUM_RADIANT_CHANNELS}, &h.radiant_start_windows[0], obj); } )
    .def("read", [](rno_g_header_t & header, py_rno_g_file_handle & handle) { return rno_g_header_read(handle.h, &header); } )
    .def("write", [](rno_g_header_t & header, py_rno_g_file_handle & handle) { return rno_g_header_write(handle.h, &header);} )
  ;


#undef FLD
#define FLD(x) .def_readonly(#x,&rno_g_waveform_t :: x)
  py::class_<rno_g_waveform_t>(m,"Waveform")
    FLD(event_number)
    FLD(run_number)
    FLD(radiant_nsamples)
    FLD(lt_nsamples)
    .def_property_readonly("radiant_waveforms", []( py::object & obj) { rno_g_waveform_t & wf = obj.cast<rno_g_waveform_t &>() ; return py::array{ {RNO_G_NUM_RADIANT_CHANNELS, RNO_G_MAX_RADIANT_NSAMPLES}, wf.radiant_waveforms,  obj}; } ) 
    .def_property_readonly("lt_waveforms", []( py::object & obj) { rno_g_waveform_t & wf = obj.cast<rno_g_waveform_t &>() ; return py::array{ {RNO_G_NUM_LT_CHANNELS, RNO_G_MAX_LT_NSAMPLES}, wf.radiant_waveforms,  obj}; } ) 
    .def("read", [](rno_g_waveform_t & waveform, py_rno_g_file_handle & handle) { return rno_g_waveform_read(handle.h, &waveform); } )
    .def("write", [](rno_g_waveform_t & waveform, py_rno_g_file_handle & handle) { return rno_g_waveform_write(handle.h, &waveform); } )
  ;


} 





