import tensorflow as tf
import os, sys

def export(model_name, micro=False):
    converter = tf.lite.TFLiteConverter.from_keras_model_file(model_name)
    if micro:
        # converter.inference_type = tf.lite.constants.QUANTIZED_UINT8
        # input_arrays = converter.get_input_arrays()
        # converter.quantized_input_stats = {input_arrays[0] : (0., 1.)}  # mean, std_dev
        converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
    tflite_model = converter.convert()
    new_fname = os.path.splitext(model_name)[0] + '.tflite'
    open(new_fname, 'wb').write(tflite_model)
    print('---------------------------------------')
    # Get size of input file
    stat_inp = os.stat(model_name)
    print('Input model size: %d bytes' % stat_inp.st_size)
    # Get size of exported file
    stat_out = os.stat(new_fname)
    print('Output model size: %d bytes' % stat_out.st_size)
    print('Compress ratio: %f' % (stat_inp.st_size/stat_out.st_size))
    print('Space savings: %f' % ( 100 - (stat_out.st_size*100.0/stat_inp.st_size)))

if __name__=="__main__":
    model_name = sys.argv[1]
    micro = True if 'micro' in sys.argv[0] else False
    if 'h5' not in model_name:
        print('Can only convert HDF5 (*.h5) models')
        quit(1)
    
    export(model_name, micro)