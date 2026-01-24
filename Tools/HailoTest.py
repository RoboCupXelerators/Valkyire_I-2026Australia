import numpy as np
from functools import partial
from hailo_platform import VDevice, HailoSchedulingAlgorithm, FormatType

def example_callback(completion_info, bindings):
    """处理推理完成的回调函数"""
    if completion_info.exception:
        print(f"推理失败: {completion_info.exception}")
        return
    
    # 获取输出结果
    output_data = bindings.output().get_buffer()
    print(f"推理完成! 输出形状: {output_data.shape}")
    print(f"输出数据类型: {output_data.dtype}")
    
    # 如果是分类任务，可以处理每个batch的结果
    if len(output_data.shape) >= 2:
        for i in range(output_data.shape[0]):
            batch_result = output_data[i]
            print(f"Batch {i+1} 结果预览: {batch_result[:5]}...")  # 显示前5个值

def batch_inference_demo():
    """演示batch=4的图像推理"""
    
    # 参数设置
    batch_size = 4
    timeout_ms = 10000
    
    # 创建VDevice
    params = VDevice.create_params()
    params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
    params.group_id = "SHARED"
    
    try:
        with VDevice(params) as vdevice:
            print("VDevice 创建成功")
            
            # 创建推理模型
            hef_path = './src/vision_utils/models/yolov8s.hef'
            infer_model = vdevice.create_infer_model(hef_path)
            print(f"模型加载成功: {hef_path}")
            
            # 设置batch size
            infer_model.set_batch_size(batch_size)
            print(f"Batch size 设置为: {batch_size}")
            
            # 查看输入输出信息
            input_shape = infer_model.input().shape
            output_shape = infer_model.output().shape
            print(f"单个样本输入形状: {input_shape}")
            print(f"单个样本输出形状: {output_shape}")
            print(f"输入形状类型: {type(input_shape)}")
            print(f"输出形状类型: {type(output_shape)}")
            
            # 设置数据格式
            infer_model.input().set_format_type(FormatType.FLOAT32)
            infer_model.output().set_format_type(FormatType.FLOAT32)
            
            # 配置推理模型
            with infer_model.configure() as configured_infer_model:
                print("模型配置完成")
                
                # 创建batch输入数据 - 修复：将list转换为tuple
                batch_input_shape = tuple([batch_size] + list(input_shape))
                print(f"Batch 输入形状: {batch_input_shape}")
                
                # 生成随机图像数据
                batch_images = np.random.rand(*batch_input_shape).astype(np.float32)
                print(f"生成了 {batch_size} 个随机图像")
                
                # 为了演示，可以给每个batch设置不同的值范围
                for i in range(batch_size):
                    # 设置不同的像素值范围，方便区分
                    batch_images[i] = np.random.rand(*input_shape).astype(np.float32) * (i + 1) * 0.2
                    print(f"Batch {i+1} 图像像素值范围: [{batch_images[i].min():.3f}, {batch_images[i].max():.3f}]")
                
                # 创建输出缓冲区 - 修复：将list转换为tuple
                batch_output_shape = tuple([batch_size] + list(output_shape))
                batch_output = np.empty(batch_output_shape).astype(np.float32)
                print(f"Batch 输出形状: {batch_output_shape}")
                
                # 创建bindings并设置缓冲区
                bindings = configured_infer_model.create_bindings()
                bindings.input().set_buffer(batch_images)
                bindings.output().set_buffer(batch_output)
                
                print("缓冲区设置完成，开始推理...")
                
                # 等待异步就绪
                configured_infer_model.wait_for_async_ready(timeout_ms=timeout_ms)
                
                # 开始异步推理
                job = configured_infer_model.run_async(
                    [bindings], 
                    partial(example_callback, bindings=bindings)
                )
                
                print("异步推理任务已提交")
                
                # 等待推理完成
                job.wait(timeout_ms)
                print("推理任务完成")
                
                # 额外的结果分析
                final_output = bindings.output().get_buffer()
                print(f"\n=== 推理结果分析 ===")
                print(f"最终输出形状: {final_output.shape}")
                print(f"输出数据类型: {final_output.dtype}")
                
                # 分析每个batch的结果
                for i in range(batch_size):
                    batch_result = final_output[i]
                    print(f"Batch {i+1}:")
                    print(f"  - 输出形状: {batch_result.shape}")
                    print(f"  - 值范围: [{batch_result.min():.6f}, {batch_result.max():.6f}]")
                    print(f"  - 均值: {batch_result.mean():.6f}")
                    
                    # 针对YOLO输出的特殊处理
                    if len(batch_result.shape) == 1 and batch_result.shape[0] == 2004:
                        print(f"  - 这是YOLO检测输出，包含边界框和置信度信息")
                        # 可以进一步解析检测结果
                        non_zero_count = np.count_nonzero(batch_result)
                        print(f"  - 非零值数量: {non_zero_count}/{len(batch_result)}")
                
    except Exception as e:
        print(f"推理过程中出现错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("=== HailoRT Batch=4 推理演示 ===")
    batch_inference_demo()