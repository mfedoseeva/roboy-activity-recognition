import React from 'react';
// import Button from "@material-ui/core/Button";
import { withStyles } from '@material-ui/core/styles';
// import { makeStyles, useTheme } from '@material-ui/core/styles';
import Drawer from '@material-ui/core/Drawer';
import CssBaseline from '@material-ui/core/CssBaseline';
import Divider from '@material-ui/core/Divider';
import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
// import ListItemIcon from '@material-ui/core/ListItemIcon';
import ListItemText from '@material-ui/core/ListItemText';
import Collapse from '@material-ui/core/Collapse';
import ExpandLess from '@material-ui/icons/ExpandLess';
import ExpandMore from '@material-ui/icons/ExpandMore';
// import Typography from '@material-ui/core/Typography';

import HARView from './har-view.js';



const drawerWidth = 240;

const styles = theme => ({
  root: {
    display: 'flex',
  },
  toolbar: theme.mixins.toolbar,
  drawer: {
    width: drawerWidth,
    flexShrink: 0,
  },
  drawerPaper: {
    width: drawerWidth,
  },
  content: {
    flexGrow: 1,
    padding: theme.spacing(3),
  },
  item: {
    backgroundColor: theme.palette.action.selected
  },
  nested: {
    paddingLeft: theme.spacing(4),
  },
});


function getItems() {
  return [
    {
      title: "Sport",
      subitems: [
        "Archery", "Bowling", "Kicking soccer ball", "Throwing ball", "Playing badminton", "Deadlifting"
      ]
    },
    {
      title: "Exercising",
      subitems: [
        "Jogging", "Squat", "Yoga", "Zumba", "Tai chi", "Capoeira", "Punching bag", "Lunge"
      ]
    },
    {
      title: "Leisure",
      subitems: [
        "Blowing out candles", "Bouncing on trampoline", "Playing cards", "Playing controller",
           "Riding mechanical bull", "Answering questions"
      ]
    },
    {
      title: "Dancing",
      subitems: [
        "Belly dancing", "Breakdancing", "Cheerleading", "Dancing ballet", "Dancing gangnam style",
           "Dancing macarena", "Robot dancing", "Swing dancing", "Tap dancing", "Tango dancing"
      ]
    },
    {
      title: "Music",
      subitems: [
        "Beatboxing", "Playing accordion", "Playing drums", "Playing flute",
         "Playing guitar", "Playing harmonica", "Playing piano", "Playing saxophone", "Singing"
      ]
    },
    {
      title: "Beauty and Hygiene",
      subitems: [
        "Applying cream", "Braiding hair", "Brushing hair", "Brushing teeth", "Curling hair",
        "Doing nails", "Filling eyebrows", "Shaving head", "Shaving legs", "Trimming or shaving beard",
        "Washing hair", "Washing hands"
      ]
    },
        {
      title: "Hobby and Profession",
      subitems: [
        "Juggling balls", "Juggling soccer ball", "Presenting weather forecast",
        "Spinning poi", "Air drumming", "Drawing", "Spray painting"
      ]
    },
    {
      title: "Movements",
      subitems: [
        "Cartwheeling", "Cracking neck", "Headbanging", "Pumping fist", "Shaking head", "Side kick",
        "Somersaulting", "Stretching arm", "Stretching leg", "High kick"
      ]
    },
    {
      title: "Hand Movements",
      subitems: [
        "Applauding", "Drumming fingers", "Finger snapping", "Rock scissors paper",
        "Sign language interpreting", "Slapping"
      ]
    },
    {
      title: "Household",
      subitems: [
        "Folding clothes", "Garbage collecting", "Ironing", "Making bed", "Sweeping floor", "Washing dishes"
      ]
    },
    {
      title: "Eating and Cooking",
      subitems: [
        "Drinking", "Drinking beer", "Drinking shots", "Eating burger", "Eating chips",
        "Eating ice cream", "Making tea", "Peeling potatoes"
      ]
    },
    {
      title: "Other",
      subitems: [
        "Counting money", "Digging", "Kissing", "Laughing", "Marching", "Shining shoes", "Smoking",
         "Sticking tongue out", "Using computer", "Whistling", "Writing", "Yawning"
      ]
    }

  ]
}


class App extends React.Component {
  state = {};
  handleClick = e => {
      this.setState({ [e]: !this.state[e] });
  };

  render() {
    const items = getItems();
    const { classes } = this.props;

    const drawer = (
      <div>
      {items.map( item => {
        return (
          <div>
            <ListItem button className={classes.item} onClick={this.handleClick.bind(this, item.title)}>
              <ListItemText primary={item.title} />
              { this.state[item.title]? <ExpandLess/> : <ExpandMore/>}
            </ListItem>
            <Collapse in={this.state[item.title]} timeout="auto" unmountOnExit>
              <List component="div" disablePadding>
                {item.subitems.map( s_item => {
                  return (
                    <ListItem button className={classes.nested}>
                      <ListItemText primary={s_item}/>
                    </ListItem>
                  );
                })}
              </List>
            </Collapse>
          </div>
        );
      })}
      </div>
    );


    return (
      <div className={classes.root}>
        <CssBaseline />
      <Drawer
        className={classes.drawer}
        variant="persistent"
        anchor="left"
        classes={{
          paper: classes.drawerPaper,
        }}
        open={true}
      >
      {drawer}
      <Divider />
      </Drawer>

        <main className={classes.content}>
          <HARView/>
        </main>
      </div>
    );

  }
}

export default withStyles(styles)(App);
