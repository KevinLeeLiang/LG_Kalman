# LG_Kalman

##### 一、给出离散时间[线性](https://so.csdn.net/so/search?q=%E7%BA%BF%E6%80%A7&spm=1001.2101.3001.7020)动态系统和三个独立的Gauss随机过程：
\begin{cases}
x_{k} & = & F_{k-1}x_{k-1}+\Gamma_{k-1}w_{k-1}\\ 
z_{k} & = & H_{k}x_{k}+v_{k}\\ 
\end{cases}

其中：  
$x_{k}$ 是$k$时刻的系统状态向量，$F_{k}$是系统状态转移矩阵，而 $w_{k}$ 是过程演化噪声，$\Gamma_{k}$ 是噪声矩阵，$z_{k}$ 是 $k$ 时刻对系统的量测向量， $H_{k}$ 是量测矩阵，而 $v_{k}$ 是量测噪声。  

${w_k}$ 是独立过程：$w_k∼N(0,Q^{v}_{k})$

${v_k}$ 是独立过程：$vk∼N(0,Q_k^v)$

系统初始状态：$x_{0}\sim N(\vec x_{0} ,P_{0})$

卡尔曼滤波算法公式推到可见：[最佳线性无偏估计BLUE](https://blog.csdn.net/haxiongha/article/details/80861538)

- 状态计算分成——–时间更新和滤波更新  
    时间更新： $\hat{x}_ {k|k−1}=F_ {k−1}\hat{x}_ {k−1}$
    
    滤波更新： $\hat{x}_ {k}=\hat{x}_ {k|k−1}+G_{k|k−1}\times\tilde{z}_ {k|k−1}$

    $G_{k|k-1}=P_{k|k-1}H^T_k[H_kP_{k|k-1}H^T_k+Q^v_k]^{-1}$

    
- 方差计算分成——-时间更新和滤波更新  
    时间更新：$P_{k|k-1}=F_{k-1}P_{k-1}F^T_{k-1}+\Gamma_{k-1}Q^{w}_ {k-1}\Gamma^{T}_ {k-1}$
    
    滤波更新：$P_k=(I−G_{k|k−1}Hk)P_{k|k−1Pk}$
    

1、 Kalman滤波理论的状态转移方程，就是离散时间系统的动态描述，又称为目标的动态模型，描述的是目标运动行为。  
2、目标的运动行为描述主要用于对目标的跟踪滤波，因此又称为目标跟踪模型。  
3、目标跟踪的数学模型  
目标跟踪的主要目的就是估计移动目标的状态轨迹。虽然目标在空间上几乎从来不是一个真正的点，但通常还是把目标看作空间没有形状的一个点，特别对于目标建模更是如此。目标动态模型描述了目标状态又随时间的演化过程。

##### 二、Kalman滤波器的应用

###### 举例1

![这里写图片描述](https://img-blog.csdn.net/20180629205846783?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2hheGlvbmdoYQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
![这里写图片描述](https://img-blog.csdn.net/20180629205922282?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2hheGlvbmdoYQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
![这里写图片描述](https://img-blog.csdn.net/20180629205948794?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2hheGlvbmdoYQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
![这里写图片描述](https://img-blog.csdn.net/20180629210515809?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2hheGlvbmdoYQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
![这里写图片描述](https://img-blog.csdn.net/20180629210643189?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2hheGlvbmdoYQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
![这里写图片描述](https://img-blog.csdn.net/20180629210716454?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2hheGlvbmdoYQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
![这里写图片描述](https://img-blog.csdn.net/20180629210800969?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2hheGlvbmdoYQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
