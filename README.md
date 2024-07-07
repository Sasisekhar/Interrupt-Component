# Interrupt-Component
This repository contains the code pertaining to the paper titled "Handling Asynchronous Inputs In A DEVS Based Real-Time Kernels". Further, the following subsections in the ReadMe document

### The DEVS Formalism
The Discrete EVent System Specification (DEVS) formalism provides a rigorous definition for discrete-event modeling and simulation. DEVS allows the user to define a mathematical object that represents an abstraction of real systems. Parallel DEVS (PDEVS) is a popular variant of the original formalism, which addresses some deficiencies of the original DEVS. In fact, multiple state-of-the-art works usually refer to PDEVS as simply DEVS. In the following, unless it is explicitly noted, the use of DEVS implies PDEVS. In DEVS, the behavior of a system can be described at two levels: atomic models, which describe the autonomous behavior of a system as a series of transitions between states and its reactions to external events, and coupled models, which describe a system as the interconnection of coupled components. [\[github.com, last accessed July 7th\]](https://github.com/SimulationEverywhere/cadmium_v2/wiki/3.-The-DEVS-Formalism)

#### Atomic Models
A DEVS model is made of Atomic models that define the basic behaviour and coupled models that define how the atomic models are grouped together.
An Atomic model _AM_ is defined as follows:
```math
AM = \langle X, Y, S, s0, ta, \delta_{int}, \delta_{ext}, \delta_{con}, \lambda \rangle
```
Where, 
- X: Set of input ports.
- Y: Set of output ports.
- S: Set of states that the model can exist in.
- s0: Initial state.
- ta: Time advance function, Advances the time of the simulation.
- $\delta_{int}$: Internal transition function, changes the state per the time advance (Endogenous)
- $\delta_{ext}$: External transition function, changes the state of the system per external input (Exogenous)
- $\delta_{con}$: Confluent transition function, decides which of the two transision functions are executed first (and by extension, decides what the next state would be) in case of a collision between $\delta_{int}$ and $\delta_{ext}$
- $\lambda$: Output function, produces the output at port y ∈Y per the internal state.

#### Coupled Models
A Coupled model _CM_ is defined as follows:
```math
CM = \langle X, Y, C, EIC, EOC, IC \rangle
```
Where,
- X: Set of input ports.
- Y: Set of output ports.
- C: Set of sub-models withinthe coupled model.
- EIC: External input coupling, defines the connections between ports in X and sub-models in C.
- EOC: External output coupling, defines the connections between ports in Y and sub-models in C.
- IC: Internal coupling, defines the connections between sub-models in C


### RT Clock

The RT Clock is an abstract class whcih can be used to implement vsrious clocks depending on the platform Cadmium is running on.

#### Abstract RT Clock
The Abstract RT Clock for Cadmimum 
```c++
//! Abstract base class of a real-time clock.
class RealTimeClock {
 protected:
    double vTimeLast;  //!< Last virtual time (i.e., the clock time in the simulation).
 public:

    RealTimeClock() : vTimeLast() {}

    virtual ~RealTimeClock() = default;

    /**
     * Virtual method for starting the real-time clock.
     * @param timeLast initial virtual time (in seconds).
     */
    virtual void start(double timeLast) {
        vTimeLast = timeLast;
    };

    /**
     * Virtual method for stopping the real-time clock.
     * @param timeLast final virtual time (in seconds).
     */
    virtual void stop(double timeLast) {
        vTimeLast = timeLast;
    }

    /**
     * Waits until the next simulation time or until an external event happens.
     * In this abstract implementation, it does nothing. Thus, it always return timeNext.
     *
     * @param nextTime next simulation time (in seconds).
     * @return next simulation time (in seconds). Return value must be less than or equal to nextTime.
     */
    virtual double waitUntil(double timeNext) {
        vTimeLast = timeNext;
        return vTimeLast;
    }
};
```
#### ESP Clock
Using the abstract RT Clock class as, the ESP Clock is written as such: (observe the 'wait_until()' method)

```c++
template<typename T = double, typename Y = uint64_t, typename Z = InterruptHandler<Y>>
class ESPclock : RealTimeClock {
private:
    gptimer_handle_t executionTimer;
    double rTimeLast;
    std::shared_ptr<Coupled> top_model;
    std::shared_ptr<InterruptHandler<Y>> ISR_handle;
    bool IE;

    void initTimer() {
        gptimer_config_t timer_config1 = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 40 * 1000 * 1000, // 10MHz, 1 tick=100ns
        };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config1, &executionTimer));

        ESP_ERROR_CHECK(gptimer_enable(executionTimer));
        gptimer_set_raw_count(executionTimer, 0);
        gptimer_start(executionTimer);
    }

 public:

    //! The empty constructor does not check the accumulated delay jitter.
    ESPclock() : RealTimeClock() {
        initTimer();
        this->top_model = NULL;
        IE = false;
    }

    [[maybe_unused]] ESPclock(std::shared_ptr<Coupled> model) : RealTimeClock() {
        initTimer();
        IE = true;
        this->top_model = model;
        this->ISR_handle = std::make_shared<Z>();
    }

    /**
     * Starts the real-time clock.
     * @param timeLast initial simulation time.
     */
    void start(double timeLast) override {
        RealTimeClock::start(timeLast);
        uint64_t count = 0;
        uint32_t res = 0;
        gptimer_get_resolution(executionTimer, &res);
        gptimer_get_raw_count(executionTimer, &count);
        rTimeLast = (double)count / (double)res;
    }

    /**
     * Stops the real-time clock.
     * @param timeLast last simulation time.
     */
    void stop(double timeLast) override {
        uint64_t count = 0;
        uint32_t res = 0;
        gptimer_get_resolution(executionTimer, &res);
        gptimer_get_raw_count(executionTimer, &count);
        rTimeLast = (double)count / (double)res;
        RealTimeClock::stop(timeLast);
    }

    /**
     * Waits until the next simulation time or until an external event happens.
     *
     * @param timeNext next simulation time (in seconds) for an internal transition.
     * @return next simulation time (in seconds). Return value must be less than or equal to timeNext.
     * */
    double waitUntil(double timeNext) override {
        auto duration = timeNext - vTimeLast;
        rTimeLast += duration;

        uint64_t count = 0;
        uint32_t res = 0;

        cadmium::Component pseudo("interrupt_block");
        cadmium::Port<Y> out;
        out = pseudo.addOutPort<Y>("out");

        gptimer_get_resolution(executionTimer, &res);
        gptimer_get_raw_count(executionTimer, &count);
        double timeNow = (double)count / (double)res;

        while(timeNow < rTimeLast) {
            gptimer_get_resolution(executionTimer, &res);
            gptimer_get_raw_count(executionTimer, &count);
            timeNow = (double)count / (double)res;

            if(IE){
                if (ISR_handle->ISRcb()) {
                    auto data = ISR_handle->decodeISR();
                    out->addMessage(data);
                    top_model->getInPort("in")->propagate(out);
                    rTimeLast = timeNow;
                    break;
                }
            }
        
        }

        return RealTimeClock::waitUntil(std::min(timeNext, timeNow));
    }
};
```

### Interrupt Handler
The interrupt handler is an abstract class that is used to set what triggeres the interrupt component. The handler also allows the Clock to retireve the data passed along through the interrupt.

#### Abstract interrupt handler
```c++
template<typename  decodeType>
    class InterruptHandler {
        
        public:

        virtual bool ISRcb() = 0;

        
        virtual decodeType decodeISR() = 0;

    };
}
```
