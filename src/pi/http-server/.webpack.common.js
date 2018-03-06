const path = require('path');

module.exports = {
  entry: './src/index.js',
  output: {
    filename: 'bundle.js',
    //path: __dirname + '/dist'
    path: path.resolve(__dirname, 'dist')
  },
  module:{
    rules:[
      {
        test:/\.[fv]s$/,
        use:'raw-loader'
      }
    ]
  }
};
