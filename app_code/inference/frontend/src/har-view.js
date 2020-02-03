import React from 'react';
import { withStyles } from '@material-ui/core/styles';
import LinearProgress from '@material-ui/core/LinearProgress';
import Paper from '@material-ui/core/Paper';
import Typography from '@material-ui/core/Typography';

const styles = theme => ({
  root: {
    display: 'flex',
    flexDirection: 'column',
    justifyContent: 'center',
    alignItems: 'center',
    width: '100%'
  },
  content: {
    flex: '1 1 auto'
  },
  imgDiv: {
    position: 'relative'
  },
  img: {
    margin: 'auto',
    display: 'block',
    maxHeight: '90vh',
    maxWidth: '100%',
  },
  textPaper: {
    position: 'absolute',
    bottom: '3%',
    left: '15%',
    width: '70%',
    textAlign: 'center',
    opacity: 0.8,
  },
  text: {
    padding: theme.spacing(2),
  },
});

const BorderLinearProgress = withStyles({
  root: {
    height: 30,
  },
})(LinearProgress);



function WebSocketClient(){
  this.autoReconnectInterval = 1000;  // ms
}

WebSocketClient.prototype.open = function(url){
  this.url = url;
  if (this.instance !== undefined && this.instance.readyState < 2) {
    console.log("non-closed socket already exists");
    return;
  }
  this.instance = new WebSocket(this.url);
  this.instance.onopen = event => {
    this.onopen();
  };
  this.instance.onmessage = event =>{
    this.onmessage(event);
  };
  this.instance.onclose = (e)=>{
    this.reconnect(e);
    this.onclose(e);
  };
  this.instance.onerror = e => {
    this.reconnect(e);
  };
}
WebSocketClient.prototype.send = function(data,option){
  try{
    this.instance.send(data,option);
  }catch (e){
    this.instance.emit('error',e);
  }
}
WebSocketClient.prototype.reconnect = function(e){
  console.log(`WebSocketClient: retry in ${this.autoReconnectInterval}ms`,e);
  var that = this;
  setTimeout(function(){
    console.log("WebSocketClient: reconnecting...");
    that.open(that.url);
  },this.autoReconnectInterval);
}
WebSocketClient.prototype.onopen = function(e){ console.log("WebSocketClient: open",arguments); }
WebSocketClient.prototype.onmessage = function(e){  console.log("WebSocketClient: message",e);  }
WebSocketClient.prototype.onerror = function(e){  console.log("WebSocketClient: error",arguments);  }
WebSocketClient.prototype.onclose = function(e){  console.log("WebSocketClient: closed",arguments); }



class HARView extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      progress: 0,
      text: "",
      img_src: "https://www.robotnik.eu/web/wp-content/uploads//2013/10/Roboy.jpg",
    }

    this.progress_timer = undefined;
    this.progress_interval = 100; // update each 100 ms
    this.progress_max = 79 * 40;
    this.progress_tick = 100 / (this.progress_max / 100);

    this.socket = new WebSocketClient()
    this.socket.open(`ws://${window.location.hostname}:8765`)

    this.socket.onclose = event => {
    }

    this.socket.onmessage = (event) => {
      let type = event.data.charAt(0);
      let msg = event.data.slice(1);

      switch(type) {
        case '0':
          this.setState({ img_src: msg });
          break;
        case '1':
          this.startProgress();
          break;
        case '3':
          this.setState({ text: msg });
          break;
        default:
          console.log(event.data);
      }
    };
  }

  startProgress() {
    if (this.state.progress_timer !== undefined) {
      clearInterval(this.progress_timer);
    }

    this.setState({ progress: 0 });

    function progress() {
      const val = Math.min(this.state.progress + this.progress_tick, 100);
      this.setState({ progress: val });
      if (this.state.progress === 100) {
        clearInterval(this.progress_timer);
        setTimeout(() => this.setState({ progress: 0 }), 700);
      }
    }

    this.progress_timer = setInterval(progress.bind(this), this.progress_interval);
  }

  componentDidMount() {
  }

  componentWillUnmount() {
    clearInterval(this.state.progress_timer);
  }

  componentDidUpdate() {
  }

  render() {
    const { classes } = this.props;

    return (
      <div className={classes.root}>
        <div className={classes.content}>
          <div className={classes.imgDiv}>
            <img className={classes.img} alt="camera" src={this.state.img_src}/>
            <Paper className={classes.textPaper}>
              <Typography variant="h2" className={classes.text}>{this.state.text}</Typography>
            </Paper>
          </div>
          <BorderLinearProgress variant="determinate" value={this.state.progress} />
        </div>
      </div>
    );
  }
}

export default withStyles(styles)(HARView);
